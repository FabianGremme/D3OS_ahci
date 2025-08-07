use alloc::alloc::alloc_zeroed;
use alloc::boxed::{Box};
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::any::Any;
use core::ptr;
use core::ptr::{addr_of_mut, null};
use log::info;
use pci_types::{BaseClass, EndpointHeader, SubClass};
use spin::RwLock;
use x86_64::structures::paging::{Page, PageTableFlags};
use x86_64::structures::paging::frame::PhysFrameRange;
use x86_64::structures::paging::page::PageRange;
use x86_64::VirtAddr;
use crate::device::ide::IdeDrive;
use crate::{pci_bus, process_manager};
use crate::memory::{frames, pages, MemorySpace, PAGE_SIZE};
use crate::memory::nvmem::NfitStructureHeader;
use crate::memory::vma::VmaType;
use crate::storage::add_block_device;
use tock_registers::registers::{InMemoryRegister, ReadOnly, ReadWrite};
use tock_registers::register_bitfields;
use tock_registers::interfaces::Writeable;
use tock_registers::interfaces::Readable;
use tock_registers::interfaces::ReadWriteable;
use crate::memory::frames::alloc;
use crate::syscall::sys_time::{sys_get_system_time, wait_ms};

const MASS_STORAGE_DEVICE: BaseClass = 0x01;
const SATA_CONTROLLER: SubClass = 0x06;

//wird verwendet, um die command engine zu starten und zu stoppen
const START: u32 = 1 << 0;
const FIS_RECIVE_ENABLE: u32 = 1 << 4;
const FIS_RECEIVE_RUNNING: u32 = 1 << 14;
const COMMAND_LIST_RUNNING: u32 = 1 << 15;

enum BiosHandoffFlags {
    BIOS_OWNED_SEMAPHORE = 1 << 0,
    OS_OWNED_SEMAPHORE = 1 << 1,
    SMI_ON_OWNERSHIP_CHANGE = 1 << 2,
    OS_OWNERSHIP_CHANGE = 1 << 3,
    BIOS_BUSY = 1 << 4
}

#[derive(Clone, Copy, Debug)]
enum DeviceSignature {
NONE = 0x00000000,
ATA = 0x00000101,
ATAPI = 0xeb140101,
ENCLOSURE_POWER_MANAGEMENT_BRIDGE = 0xc33c0101,
PORT_MULTIPLIER = 0x96690101
}

#[allow(warnings)]
struct AhciController{
    hba_regs: HBARegister,
    ports: Vec<HbaPort>,
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HBARegister{
     hostCapabilities: u32,
     globalHostControl: u32,
     interruptStatus: u32,
     portsImplemented: u32,
     version: u32,
     commandCompletionCoalescingControl: u32,
     commandCompletionCoalescingPorts: u32,
     enclosureManagementLocation: u32,
     enclosureManagementControlu: u32,
     extendedHostCapabilities: u32,
     biosHandoffControl: u32,
     reserved: [u8;116],
     vendorSpecific: [u8;96],
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaPort {
     commandListBaseAddress: u32,
     commandListBaseAddressUpper: u32,
     fisBaseAddress: u32,
     fisBaseAddressUpper: u32,
     interruptStatus: u32,
     interruptEnable: u32,
     command: u32,
     reserved1: u32,
     taskFileData: u32,
     signature: u32,
     sataStatus: u32,
     sataControl: u32,
     sataError: u32,
     sataActive: u32,
     commandIssue: u32,
     sataNotification: u32,
     fisBasedSwitchControl: u32,
     deviceSleep: u32,
     reserved2: [u32;10],
     vendorSpecific: [u32;4],
}

/*
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaCommandHeader {
    // DWORD 0
    uint8_t commandFisLength: 5;
    uint8_t atapi: 1;
    uint8_t write: 1;
    uint8_t prefetchable: 1;

    uint8_t reset: 1;
    uint8_t bist: 1;
    uint8_t clearBusyOnOK: 1;
    uint8_t reserved1: 1;
    uint8_t portMultiplierPort: 4;

    physicalRegionDescriptorTableLength: u16,

    // DWORD 1
    physicalRegionDescriptorByteCount: u32,

    // DWORD 2-3
    commandTableDescriptorBaseAddress: u32,
    commandTableDescriptorBaseAddressUpper: u32,

    // DWORD 4-7
    reserved: [u32;4],
}*/

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaCommandTableHeader {
    // DWORD 0
    first: u32, //ReadWrite<u32, D0::Register>,

    // DWORD 1
    physicalRegionDescriptorByteCount: u32,

    // DWORD 2-3
    commandTableDescriptorBaseAddress: u32,
    commandTableDescriptorBaseAddressUpper: u32,

    // DWORD 4-7
    reserved: [u32;4],
}


//Laut Bachelorarbeit soll eine combined HBA Command Table aus einer cmd_table und 8 Einheiten der Liste entstehen
#[allow(warnings)]
#[derive(Debug)]
struct combined_HBA_CommandTable{
    cmd_table: HbaCommandTable,
    physicalRegionDescriptorTable: Vec<HbaPhysicalRegionDescriptorTableEntry>,
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaPhysicalRegionDescriptorTableEntry {
    dataBaseAddress: *mut u32,
    dataBaseAddressUpper: * mut u32,
    reserved1: u32,
    rest: u32,

    //uint32_t dataByteCount: 22;
    //uint32_t reserved2: 9;
    //uint32_t interruptOnCompletion: 1;
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct HbaCommandTable {
    commandFis: [u8;64],
    atapiCommand: [u8;16],
    reserved: [u8;48],
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug)]
struct DeviceInfo {
    config: u16,                /* lots of obsolete bit flags */
    cyls: u16,                  /* obsolete */
    reserved2: u16,             /* special config */
    heads: u16,                 /* "physical" heads */
    track_bytes: u16,           /* unformatted bytes per track */
    bytesPerSector: u16,        /* unformatted bytes per sector */
    sectors: u16,               /* "physical" sectors per track */
    vendor0: u16,               /* vendor unique */
    vendor1: u16,               /* vendor unique */
    vendor2: u16,               /* vendor unique */
    serialNumber: [u8;20],       /* 0 = not specified */
    buf_type: u16,
    buf_size: u16,              /* 512 byte increments; 0 = not specified */
    ecc_bytes: u16,             /* for r/w long cmds; 0 = not specified */
    firmwareRevision: [u8;8],    /* 0 = not specified */
    Port: [u8;40],              /* 0 = not specified */
    multi_count: u16,           /* Multiple Count */
    dword_io: u16,              /* 0=not_implemented; 1=implemented */
    capability1: u16,           /* vendor unique */
    capability2: u16,           /* bits 0:DMA 1:LBA 2:IORDYsw 3:IORDYsup word: 50 */
    vendor5: u8,                /* vendor unique */
    tPIO: u8,                   /* 0 = slow, 1 = medium, 2 = fast */
    vendor6: u8,                /* vendor unique */
    tDMA: u8,                   /* 0 = slow, 1 = medium, 2 = fast */
    field_valid: u16,           /* bits 0:cur_ok 1:eide_ok */
    cur_cyls: u16,              /* logical cylinders */
    cur_heads: u16,             /* logical heads word 55 */
    cur_sectors: u16,           /* logical sectors per track */
    cur_capacity0: u16,         /* logical total sectors on drive */
    cur_capacity1: u16,         /* (2 words, misaligned int)     */
    multsect: u8,               /* current multiple sector count */
    multsect_valid: u8,         /* when (bit0==1) multsect is ok */
    lbaCapacity: u32,           /* total number of sectors */
    dma_1word: u16,             /* single-word dma info */
    dma_mword: u16,             /* multiple-word dma info */
    eide_pio_modes: u16,        /* bits 0:mode3 1:mode4 */
    eide_dma_min: u16,          /* min mword dma cycle time (ns) */
    eide_dma_time: u16,         /* recommended mword dma cycle time (ns) */
    eide_pio: u16,              /* min cycle time (ns), no IORDY */
    eide_pio_iordy: u16,        /* min cycle time (ns), with IORDY */
    words69_70: [u16;2],        /* reserved words 69-70 */
    words71_74: [u16;4],        /* reserved words 71-74 */
    queue_depth: u16,
    sata_capability: u16,       /* SATA Capabilities word 76 */
    sata_additional: u16,       /* Additional Capabilities */
    sata_supported: u16,        /* SATA Features supported */
    features_enabled: u16,      /* SATA features enabled */
    major_rev_num: u16,         /* Major rev number word 80 */
    minor_rev_num: u16,         /* Minor revision number */
    command_set_1: u16,         /* bits 0: Smart, 1: Security, 2: Removable, 3: PM */
    command_set_2: u16,         /* bits 14:Smart Enabled 13:0 zero */
    cfsse: u16,                 /* command set-feature supported extensions */
    cfs_enable_1: u16,          /* command set-feature enabled */
    cfs_enable_2: u16,          /* command set-feature enabled */
    csf_default: u16,           /* command set-feature default */
    dma_ultra: u16,
    word89: u16,                /* reserved (word 89) */
    word90: u16,                /* reserved (word 90) */
    CurAPMvalues: u16,          /* current APM values */
    word92: u16,                /* reserved (word 92) */
    comreset: u16,              /* should be cleared to 0 */
    accoustic: u16,             /*  accoustic management */
    min_req_sz: u16,            /* Stream minimum required size */
    transfer_time_dma: u16,     /* Streaming Transfer Time-DMA */
    access_latency: u16,        /* Streaming access latency-DMA & PIO WORD 97*/
    perf_granularity: u32,      /* Streaming performance granularity */
    total_usr_sectors: [u32;2],       /* Total number of user addressable sectors */
    transfer_time_pio: u16,     /* Streaming Transfer time PIO */
    reserved105: u16,           /* Word 105 */
    sector_sz: u16,             /* Physical Sector size / Logical sector size */
    inter_seek_delay: u16,      /* In microseconds */
    words108_116: [u16;9],            /* Reserved */
    words_per_sector: u32,      /* words per logical sectors */
    supported_settings: u16,    /* continued from words 82-84 */
    command_set_3: u16,         /* continued from words 85-87 */
    words121_126: [u16;6],            /* reserved words 121-126 */
    word127: u16,               /* reserved (word 127) */
    security_status: u16,       /* device lock function
                                         * 15:9   reserved
                                         * 8   security level 1:max 0:high
                                         * 7:6   reserved
                                         * 5   enhanced erase
                                         * 4   expire
                                         * 3   frozen
                                         * 2   locked
                                         * 1   en/disabled
                                         * 0   capability */
    csfo: u16,                 /* current set features options
                                         * 15:4   reserved
                                         * 3   auto reassign
                                         * 2   reverting
                                         * 1   read-look-ahead
                                         * 0   write cache */
    words130_155: [u16;26],          /* reserved vendor words 130-155 */
    word156: u16,
    words157_159: [u16;3],            /* reserved vendor words 157-159 */
    cfa: u16,                   /* CFA Power mode 1 */
    words161_175: [u16;15],           /* Reserved */
    media_serial: [u8;60],            /* words 176-205 Current Media serial number */
    sct_cmd_transport: u16,     /* SCT Command Transport */
    words207_208: [u16;2],            /* reserved */
    block_align: u16,           /* Alignement of logical blocks in larger physical blocks */
    WRV_sec_count: u32,         /* Write-Read-Verify sector count mode 3 only */
    verf_sec_count: u32,        /* Verify Sector count mode 2 only */
    nv_cache_capability: u16,   /* NV Cache capabilities */
    nv_cache_sz: u16,           /* NV Cache size in logical blocks */
    nv_cache_sz2: u16,          /* NV Cache size in logical blocks */
    rotation_rate: u16,         /* Nominal media rotation rate */
    word218: u16,               /* Reserved  */
    nv_cache_options: u16,      /* NV Cache options */
    words220_221: [u16;2],            /* reserved */
    transport_major_rev: u16,
    transport_minor_rev: u16,
    words224_233: [u16;10],           /* Reserved */
    min_dwnload_blocks: u16,    /* Minimum number of 512 byte units per DOWNLOAD MICROCODE command for mode 03h */
    max_dwnload_blocks: u16,    /* Maximum number of 512 byte units per DOWNLOAD MICROCODE command for mode 03h */
    words236_254: [u16;19],          /* Reserved */
    integrity: u16,             /* Cheksum, Signature */
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct FisRegisterHostToDevice {
    // DWORD 0
    typ: u8,
    combined: u8,

    //uint8_t portMultiplierPort: 4;
    //uint8_t reserved1: 3;
    //uint8_t commandControl: 1;

    command: u8,
    featureLow: u8,

    // DWORD 1
    lba0: u8,
    lba1: u8,
    lba2: u8,
    device: u8,

    // DWORD 2
    lba3: u8,
    lba4: u8,
    lba5: u8,
    featureHigh: u8,

    // DWORD 3
    countLow: u8,
    countHigh: u8,
    isochronousCommandCompletion: u8,
    control: u8,

    // DWORD 4
    reserved2: u32,
}


#[allow(warnings)]
pub fn init(){
    info!("searching the bus for mass storage devices that use sata");
    let mut found_devices = pci_bus().search_by_class(MASS_STORAGE_DEVICE as BaseClass, SATA_CONTROLLER as SubClass);
    info!("habe die folgenden Geräte gefunden {:?}", found_devices.len());
    let mut device = found_devices.pop().unwrap();
    unsafe {
        let mut ahci_controller = Arc::new(AhciController::new(device));
        info!("der ahci controller hat die hba: {:?}", ahci_controller.hba_regs);
        ahci_controller.check_bios_handoff();
        ahci_controller.check_ports_for_device();
        ahci_controller.check_ahci_mode_enabled();
        ahci_controller.check_only_ahci();
        ahci_controller.check_64_bit_addr_supported();
        ahci_controller.check_cap_nr_of_ports();
        ahci_controller.check_nr_of_command_slots();
        ahci_controller.map_command_components();
        //info!("teste die Funktion um mehrere Bitfelder auszulesen");
        //let testoutput = ahci_controller.general_bitlen_reader(57105, 7, 5); // hier sollte 30 rauskommen, das passt
        //info!("testoutput ist {}", testoutput);

        info!("before cmd");
        ahci_controller.test_ports_command_engine();
        info!("after cmd");
        ahci_controller.find_slot_all_ports();

        let mut var:Box<u32> = Box::new(4000);
        info!("size of var ist {:?}", size_of_val(&var));
        let mut pointer: *mut u32 = Box::into_raw(var);
        info!("pointer ist bei {:?}", pointer);
        let test_cmd_table = ahci_controller.create_combined_hba_cmd_table(1, pointer);
        info!("die erzeugte combined cmd table ist {:?}", test_cmd_table);
        let testregion = AhciController::allocate_heap_region(40);
        info!("testregion im heap ist {:?}", testregion);

        let id_device = ahci_controller.identify_device(0);
        info!("id device is {:?}", id_device);


        //let model_nr = id_device.clone();
        let serial_nr = id_device.serialNumber.clone();
        let serial_str = String::from_utf8(Vec::from(serial_nr)).unwrap();
        let firmware_rev = id_device.firmwareRevision.clone();
        let firmware_str = String::from_utf8(Vec::from(firmware_rev)).unwrap();

        info!("die neue serial nr ist {}, und die neue firmware ist {}", serial_str, firmware_str);
    }



    //die GHCR sind in Section 3 der Spezifikation zu finden. ich weiß noch nicht, wie man bis dahin kommt
}
#[allow(warnings)]
impl AhciController {

    unsafe fn fill_hba_reg(ahci_base_addr: *mut u8) -> HBARegister{
        let cap = ahci_base_addr as *mut u32;
        let ghc = ahci_base_addr.offset(4 as isize) as *mut u32;
        let is = ahci_base_addr.offset(8 as isize) as *mut u32;
        let pi = ahci_base_addr.offset(12 as isize) as *mut u32;
        let vs = ahci_base_addr.offset(16 as isize) as *mut u32;

        let cccc = ahci_base_addr.offset(20 as isize) as *mut u32;
        let cccp = ahci_base_addr.offset(24 as isize) as *mut u32;
        let eml = ahci_base_addr.offset(28 as isize) as *mut u32;
        let emc = ahci_base_addr.offset(32 as isize) as *mut u32;
        let ehc = ahci_base_addr.offset(36 as isize) as *mut u32;
        let bhc = ahci_base_addr.offset(40 as isize) as *mut u32;
        HBARegister {
            hostCapabilities: cap.read(),
            globalHostControl: ghc.read(),
            interruptStatus: is.read(),
            portsImplemented: pi.read(),
            version: vs.read(),
            commandCompletionCoalescingControl: cccc.read(),
            commandCompletionCoalescingPorts: cccp.read(),
            enclosureManagementLocation: eml.read(),
            enclosureManagementControlu: emc.read(),
            extendedHostCapabilities: ehc.read(),
            biosHandoffControl: bhc.read(),
            reserved: [0;116],
            vendorSpecific: [0;96],
        }
    }

    unsafe fn init_ports(ahci_base_addr: *mut u8, hba_ports: u32) ->Vec<HbaPort>{
        //aus der hba ports variable muss erst mal die Anzahl der Ports bestimmt werden. Dazu muss die Anzahl der 1 in der Binaerform gezaehlt werden.
        let mut port_nr = 0;
        let mut calc = hba_ports;
        while calc != 0{
            calc = calc & (calc -1);
            port_nr += 1;
        }
        info!("port anzahl = {:?}", port_nr);
        let mut output : Vec<HbaPort> = Vec::<HbaPort>::new();
        for i in 0..port_nr{
            output.push(Self::fill_port(ahci_base_addr,i));
        }
        output
    }

    unsafe fn fill_port(ahci_base_addr: *mut u8, nr_of_port: u64) -> HbaPort{
        let mut port_offset = (256 + (nr_of_port * 128))  as isize;
        let clb = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let clbu = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let fis_ba = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let fis_bau = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let istat = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let ie = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let cmd = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let res1 = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let tfd = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let sig = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let sata_stat = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let sata_ctrl = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let sata_err = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let sata_act = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let cmd_issue = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let sata_not = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let fis_bsc = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;
        let dev_sleep = ahci_base_addr.offset(port_offset) as *mut u32;
        port_offset += 4;


        let output = HbaPort{
            commandListBaseAddress: clb.read(),
            commandListBaseAddressUpper: clbu.read(),
            fisBaseAddress: fis_ba.read(),
            fisBaseAddressUpper: fis_bau.read(),
            interruptStatus: istat.read(),
            interruptEnable: ie.read(),
            command: cmd.read(),
            reserved1: res1.read(),
            taskFileData: tfd.read(),
            signature: sig.read(),
            sataStatus: sata_stat.read(),
            sataControl: sata_ctrl.read(),
            sataError: sata_err.read(),
            sataActive: sata_act.read(),
            commandIssue: cmd_issue.read(),
            sataNotification: sata_not.read(),
            fisBasedSwitchControl: fis_bsc.read(),
            deviceSleep: dev_sleep.read(),
            reserved2: [0;10],
            vendorSpecific: [0;4],
        };
        info!("bearbeite Port Nr {:?} mit den Feldern {:?}", nr_of_port, output);
        output
    }

    // muss das nicht command list header sein?

    unsafe fn fill_cmd_table_header(start: *mut u8) ->HbaCommandTableHeader{
        let dword0 = start as *mut u32;
        let mut offset = 4;
        let dword1 = start.offset(offset) as *mut u32;
        offset += 4;
        let dword2 = start.offset(offset) as *mut u32;
        offset += 4;
        let dword3 = start.offset(offset) as *mut u32;
        offset += 4;
        let dword4 = start.offset(offset) as *mut u32;
        offset += 4;
        let dword5 = start.offset(offset) as *mut u32;
        offset += 4;
        let dword6 = start.offset(offset) as *mut u32;
        offset += 4;
        let dword7 = start.offset(offset) as *mut u32;

        info!("dword0 = {:?}, dword1 = {:?}, dword2 = {:?}, dword3 = {:?}, dword4 = {:?}, dword5 = {:?}, dword6 = {:?}, dword7 = {:?}"
                ,dword0.read(), dword1.read(), dword2.read(), dword3.read(), dword4.read(), dword5.read(), dword6.read(), dword7.read());



        HbaCommandTableHeader{
            // DWORD 0
            first: dword0.read(),

            // DWORD 1
            physicalRegionDescriptorByteCount: dword1.read(),

            // DWORD 2-3
            commandTableDescriptorBaseAddress: dword2.read(),
            commandTableDescriptorBaseAddressUpper: dword3.read(),

            // DWORD 4-7
            reserved: [dword4.read(), dword5.read(), dword6.read(), dword7.read()],
        }
    }

    unsafe fn new(device: &RwLock<EndpointHeader>) -> Self {
        let device_header = device.read();

        // bei base address register (bar5) stehen die wichtigen Daten für die pci capabilities, register, etc.
        let bar5 = device_header.bar(5,&pci_bus().config_space());
        // bei bar4 findet sich ein io port
        let bar4 = device_header.bar(4,&pci_bus().config_space());
        info!("bar with slot one has the following info: {:?}", bar5);
        let bar_io = bar4.unwrap().unwrap_io();
        let bar_mem = bar5.unwrap().unwrap_mem();
        info!("bar io is {:?} and bar mem is {:?}", bar_io, bar_mem);

        let ahci_base_addr = bar_mem.0 as *mut u8;

        //map the memory where the control registers are located
        Self::map_general(bar_mem.0 as u64, bar_mem.1 as u64, "ahci");
        let hba = Self::fill_hba_reg(ahci_base_addr);

        Self{
            hba_regs: hba,
            ports:Self::init_ports(ahci_base_addr,hba.portsImplemented)
        }

    }


    //length is in bytes
    pub unsafe fn map_general(address: u64, length: u64, tag: &str){
        info!(
                "Found non-volatile memory (Address: [0x{:x}], Length: [{} B])",
                address,
                length
            );

        info!("length is {} and num_pages is {}", length, length/PAGE_SIZE as u64);
        let process = process_manager()
            .read()
            .kernel_process()
            .expect("Failed to get kernel process");

        // Map non-volatile memory range to kernel address space
        let start_page = pages::page_from_u64(address).expect("address is not page aligned");
        let start_page_frame = frames::frame_from_u64(address).expect("address is not page aligned");

        let test = PhysFrameRange {
            start: start_page_frame,
            end: start_page_frame + ((length + PAGE_SIZE as u64 -1)  / PAGE_SIZE as u64)};
        info!("testframe is {:?}", test);

        // Allocate virtual memory area for the non-volatile memory
        let vma = process.virtual_address_space.alloc_vma(
            Some(start_page),
            length / PAGE_SIZE as u64,
            MemorySpace::Kernel,
            VmaType::DeviceMemory,
            tag,
        ).expect("alloc_vma failed");

        // Map non-volatile memory to the kernel address space
        process.virtual_address_space.map_pfr_for_vma(
            &vma,
            PhysFrameRange {
                start: start_page_frame,
                end: start_page_frame + (length / PAGE_SIZE as u64),
            },
            PageTableFlags::PRESENT | PageTableFlags::WRITABLE,
        ).expect("map_pfr_for_vma failed for NVRAM");
    }

    pub fn general_bit_check(register: u32, bit_position: u8)->bool{
        let mask = 1<<bit_position;
        return register & mask != 0;
    }

    pub fn general_bitlen_reader(register: u32, bit_position: u8, len: u8)-> u32{
        let mut mask = 1<<bit_position;
        for i in 0..len{
            mask = mask | 1<<(bit_position + i)
        }
        return (register & mask)>>bit_position;
    }

    pub fn translate_signature(sign: u32)->DeviceSignature{
        match sign {
            0x00000000 => return DeviceSignature::NONE,
            0x00000101 => return DeviceSignature::ATA,
            0xeb140101 => return DeviceSignature::ATAPI,
            0xc33c0101 => return DeviceSignature::ENCLOSURE_POWER_MANAGEMENT_BRIDGE,
            0x96690101 => return DeviceSignature::PORT_MULTIPLIER,
            _ => {
                info!("value not found");
                return DeviceSignature::NONE
            }
        }
    }

    pub fn check_ports_for_device(& self){
        for current_port in &self.ports{
            if Self::check_port_usable(current_port.clone()){
                let signature = current_port.signature;
                info!("the device signature is {:?}", Self::translate_signature(signature));

            }

        }
    }

    pub fn check_port_usable(port:HbaPort)-> bool{
        let ssts = port.sataStatus;
        let ipm = (ssts >> 8) & 0x0F;
        let det = ssts & 0x0F;

        if ipm != 0x01 {    //0x01 means that the interface of the device is active. only then the device can be accessed
            //info!("ERR: interface is not active");
            return false;
        }
        if det != 0x03 {    //0x03 means that the device is detected and a physical communication is established
            //info!("ERR: device is not detected, or physical communication not established");
            return false;
        }
        true
    }

    pub fn check_ahci_mode_enabled(&self){
        let ghc = self.hba_regs.globalHostControl;
        let output = Self::general_bit_check(ghc, 31);
        if output{
            info!("der Controller läuft im ahci modus");
        }else{
            info!("der Controller läuft nicht im ahci modus");
        }
    }

    pub fn check_only_ahci(&self){
        let sam = self.hba_regs.hostCapabilities;
        let output = Self::general_bit_check(sam, 18);
        if output{
            info!("der Controller unterstützt nur ahci");
        }else{
            info!("der Controller unterstützt nicht nur ahci");
        }
    }

    pub fn check_bios_handoff(&self){
        //check if the version is high enough
        if self.hba_regs.version >= 0x10200{
            info!("Version ist hoch genug");
            let ext_cap = self.hba_regs.extendedHostCapabilities;
            info!("ext_cap sind {}", ext_cap);
            if ext_cap & 1 != 0{
                info!("BIOS Handoff wird vom Controller unterstützt")
            }
        }else{
            info!("Version ist nicht hoch genug")
        }
        let handoff = self.hba_regs.biosHandoffControl;
        if handoff == 0{
            info!("the bios has no control over the hba, so the os can use it");
        }
    }

    pub fn check_64_bit_addr_supported(&self){
        let cap = self.hba_regs.hostCapabilities;
        let output = Self::general_bit_check(cap, 31);
        if output{
            info!("es werden 64 bit adressen unterstützt");
        }else{
            info!("es werden 32 bit adressen unterstützt");
        }

    }

    pub fn check_cap_nr_of_ports(&self){
        let cap = self.hba_regs.hostCapabilities;
        let nr_of_ports = Self::general_bitlen_reader(cap, 0, 5);
        info!("laut capabilities werden {} Ports unterstützt.", nr_of_ports);
    }


    pub fn check_nr_of_command_slots(&self)->u32{
        let cap = self.hba_regs.hostCapabilities;
        let nr_of_cmds = Self::general_bitlen_reader(cap, 8, 5);
        info!("laut capabilities werden {} Command slots unterstützt.", nr_of_cmds);
        nr_of_cmds
    }

    pub fn map_command_components(&self){
        info!("self.ports ist: {:?}", self.ports);
        for port in &self.ports{
            if Self::check_port_usable(port.clone()){
                self.map_command_for_port(*port);
            }

        }

    }
    // es werden drei Strukturen gemappt:
    // die Command list structure besteht aus 32 Command headern. jeder header besteht aus 4 Dwords und 4 reserved Dwords
    // die Region für received fis werden direkt aus dem Port gelesen und hier können von eingehenden Fis Werte geschrieben werden
    // jeder header innerhalb der command list verweist auf eine eigene command table, in der command fis, atapi command und physical region descriptor table liegen

    pub fn map_command_for_port(&self, port: HbaPort){
        self.stop_cmd_engine(port);
        //baue die Adresse für die 32 cmd header
        // die header zusammen bilden die command list
        let first_cmd_header_addr:u64 = port.commandListBaseAddress as u64 | ((port.commandListBaseAddressUpper as u64) << 32);
        let size_cmd_header = 1024;

        //baue die Adresse für die received FIS
        let received_fis: u64 = port.fisBaseAddress as u64 | ((port.fisBaseAddressUpper as u64) << 32);
        let size_received_fis = 256;
        info!("die Addressen sind: cmd_header: {:x}, received_fis: {:x}", first_cmd_header_addr, received_fis);
        unsafe {
            //falls zwei memory spaces auf je kleiner als eine Seite sind, wird nach den Startadressen abhängig gemacht,
            // ob sie spaces sich die Page teilen, oder separate Pages erhalten

            if received_fis - first_cmd_header_addr >= PAGE_SIZE as u64{
                Self::map_general(first_cmd_header_addr, PAGE_SIZE as u64, "cmd_hd");
                Self::map_general(received_fis, PAGE_SIZE as u64, "rc_fis");
            }else {
                // cmd header ist vor page size. beide sind kleiner als eine Page, also teilen sie sich zwei Seiten
                Self::map_general(first_cmd_header_addr, 2 * PAGE_SIZE as u64, "cmd_and_fis");
            }

            //test if the cmd_List has a
            let cmd_header1 = Self::fill_cmd_table_header(first_cmd_header_addr as *mut u8);
            info!("the first command header struct has the following values: {:?}", cmd_header1);
            // hier werden die einzelnen Werte von cmd_header1 wie prdt, etc ausgeschrieben
            let full_register = cmd_header1.first;
            let prdt = AhciController::general_bitlen_reader(full_register, 16, 16);
            info!("prdt ist {:?}", prdt);

            //map the first command table with the info of the first command header
            let first_cmd_table_addr = cmd_header1.commandTableDescriptorBaseAddress as u64 | ((cmd_header1.commandTableDescriptorBaseAddressUpper as u64)<<32);
            info!("the first_cmd_table_addr is {:x}", first_cmd_table_addr);
            // das ist die Größe aus der combined command table mit 8 Inhalten
            let cmd_table_size = 256;
            Self::map_general(first_cmd_table_addr, PAGE_SIZE as u64, "cmd_tbl");


            //check, if the first command table has actual values inside it

            self.start_cmd_engine(port);

        }

    }

    pub fn test_ports_command_engine(&self){
        for port in &self.ports{
            if Self::check_port_usable(port.clone()){
                info!("befor single start");
                self.start_cmd_engine(*port);
                info!("after single start");
                self.stop_cmd_engine(*port);
                info!("after single stop");
            }

        }

    }
    pub fn start_cmd_engine(&self, mut port: HbaPort){
        info!("port ist nun {:?}", port);
        while(port.command & COMMAND_LIST_RUNNING) > 0{
            wait_ms(10);
        }
        port.command |= (START | FIS_RECIVE_ENABLE);
        info!("port ist nun {:?}", port);
    }

    pub fn stop_cmd_engine(&self, mut port: HbaPort){
        info!("port ist nun {:?}", port);
        port.command &= (START | FIS_RECIVE_ENABLE);
        while (port.command & (FIS_RECEIVE_RUNNING | COMMAND_LIST_RUNNING)) > 0{
            wait_ms(10);
        }
        info!("port ist nun {:?}", port);
    }


    //finden eines freien command headers über den port
    pub fn find_cmd_slot(&self, mut port: HbaPort) -> i32{
        let nr_cmd_slots = self.check_nr_of_command_slots();
        let mut slots = port.sataActive | port.sataError;
        info!("slots ist {:b}", slots);
        for i in 0..nr_cmd_slots{
            if (slots & 1) == 0{
                info!("slot gefunden an Stelle {}", i);
                return i as i32;
            }
            slots >>=1;
        }
        info!("kein Slot gefunden!");
        -1
    }

    pub fn find_slot_all_ports(&self) -> Option<HbaPort> {
        for port in &self.ports{
            if Self::check_port_usable(port.clone()){
                if (self.find_cmd_slot(*port)) != -1{
                    return Some(*port);

                }
            }
        }
        None

    }

    // fis steht für frame information structure

    pub fn identify_device(&self, portnr: u32) -> DeviceInfo{
        let mut command_fis = [0u8;64];
        let mut atapi_cmd = [0u8;16];

        //prepare the host to device fis (muss das nicht mehr gesendet werden??)
        // das muss noch in den command fis gelegt werden
        let mut host_to_device_fis = FisRegisterHostToDevice{
            typ: 39,
            combined: 1,
            command: 0,
            featureLow: 0,
            lba0: 0,
            lba1: 0,
            lba2: 0,
            device: 0,
            lba3: 0,
            lba4: 0,
            lba5: 0,
            featureHigh: 0,
            countLow: 0,
            countHigh: 0,
            isochronousCommandCompletion: 0,
            control: 0,
            reserved2: 0,
        };

        let port = self.ports[portnr as usize];
        if port.signature == 257{                           //port signature if it is an ata port
            host_to_device_fis.command = 236;               //identification code for ata
        }else{
            host_to_device_fis.command = 161;               //identification code for atapi
        }
        info!("found port is {:?}", port);
        info!("host_to_device_fis is {:?}", host_to_device_fis);
        //info!("old command fis is now {:?}", command_fis);
        // copy the info from the struct into the memory region
        unsafe {
            let ptr = command_fis.as_mut_ptr();
            let src = &host_to_device_fis as *const FisRegisterHostToDevice as *const u8;
            ptr::copy_nonoverlapping(src, ptr, size_of::<FisRegisterHostToDevice>());
        }
        //info!("new command fis is now {:?}", command_fis);

        let mut info = self.read_from_device(portnr,512, command_fis, atapi_cmd).unwrap();
        let mut info_ptr = addr_of_mut!(info).addr();
        let mut output = info_ptr as *mut DeviceInfo;
        unsafe {
            output.read()
        }
    }

    //Befehl für identify device:
    /*AhciController::DeviceInfo* AhciController::identifyDevice(uint32_t portNumber) {
    uint8_t commandFis[64]{};
    uint8_t atapiCommand[16]{};

    //hier wird das Fis zusammengebaut
    auto &hostToDeviceFis = reinterpret_cast<FisRegisterHostToDevice>(commandFis);
    hostToDeviceFis.type = REGISTER_HOST_TO_DEVICE;
    hostToDeviceFis.commandControl = 1;
    hostToDeviceFis.command = registers->ports[portNumber].signature == ATA ? ATA_IDENTIFY : ATAPI_IDENTIFY;

    //hier wird das abgesendet
    auto info = static_cast<DeviceInfo>(readFromDevice(portNumber, 512, commandFis, atapiCommand));
    //info ist wohl hier das device info, welches man aus readFromDevice bekommen hat
    if (info != nullptr) {
    Auslesen der info Werte
    byteSwapString(reinterpret_cast<char>(info->model), sizeof(DeviceInfo::model));
    byteSwapString(reinterpret_cast<char>(info->serialNumber), sizeof(DeviceInfo::serialNumber));
    byteSwapString(reinterpret_cast<char*>(info->firmwareRevision), sizeof(DeviceInfo::firmwareRevision));
    }

    return info;
    }*/


    //innerhalb der clb gibt es eine command liste
    pub fn read_from_device(&self, portnr: u32, byte_count: u32, mut command_fis:[u8;64], atapi_command: [u8;16]) -> Option<Vec<u32>> {
        let port = self.ports[portnr as usize];
        info!("port in read from device ist {:?}", port);
        let mut command_list_addr = port.commandListBaseAddress as u64 | ((port.commandListBaseAddressUpper as u64) << 32);
        unsafe{
            // weil ich nur bisher einen cmd_header in der Liste habe, kann ich da direkt reinschreiben
            let mut first_cmd_header = Self::fill_cmd_table_header(command_list_addr as *mut u8);
            //die command List besteht aus cmd_table_headern, welche selbst dann auf die command Table verweisen

            // hier noch ein paar Hilfen
            if Self::check_port_usable(port) !=true{
                info!("ERR: Port is not usable");
                return None;
            }

            let slot = self.find_cmd_slot(port);
            if slot == -1{
                info!("ERR: Slot nicht gefunden");
                return None;
            }

            // hier soll dann der DMA Buffer impl werden
            let mut dma_reg = AhciController::allocate_heap_region(byte_count);
            let dma_reg_addr = dma_reg.as_mut_ptr();

            //Hier fragen: müsste ich nicht einfach auch damit durchkommen?

            // hier werden dann die Werte kopiert
            // hier wird nur die command table gemacht, nicht die command lsit
            let mut combined_cmd_table = self.create_combined_hba_cmd_table(byte_count, dma_reg_addr);
            combined_cmd_table.cmd_table.commandFis = command_fis.clone();
            combined_cmd_table.cmd_table.atapiCommand = atapi_command.clone();

            // jetzt wird in die command list geschrieben

            let mut physical_region_descriptor_table_length = byte_count / 4096;
            if physical_region_descriptor_table_length == 0{
                physical_region_descriptor_table_length = (byte_count / 4096) +1;
            }
            //nachschauen, wie ich auf diese Größen komme
            let mut cmd_fis_len = size_of::<FisRegisterHostToDevice>() / size_of::<u32>();
            let mut atapi = 0;
            if atapi_command[0] != 0{
                atapi = 1;
            }
            // teste ob addr_of_mut funktioniert
            let cmd_table_base_addr: u64 = addr_of_mut!(combined_cmd_table).addr() as u64;
            let upper_cmd_table_base_addr: u32 = (cmd_table_base_addr >> 32) as u32;
            let lower_cmd_table_base_addr = cmd_table_base_addr as u32;

            //alles zu dem first zusammenfügen (atapi, cmd_fis_len und prdt_len)
            // teste ob first als binary richtig ausgefüllt wird
            let combined = (physical_region_descriptor_table_length << 16) as u32 | ( atapi << 5) as u32 | cmd_fis_len as u32;
            first_cmd_header.first = combined;

            first_cmd_header.commandTableDescriptorBaseAddressUpper = upper_cmd_table_base_addr;
            first_cmd_header.commandTableDescriptorBaseAddress = lower_cmd_table_base_addr;

            // hier wären noch ein paar Fehlerabfragen

            Some(dma_reg)
        }

    }

    //vorher muss die read from device implementiert werden:
    /*
    void* AhciController::readFromDevice(uint32_t portNumber, uint32_t byteCount, const uint8_t commandFis[64], const uint8_t atapiCommand[16]) {

    //suche den richtigen Port und richtige command list
    auto &memoryService = Kernel::Service::getService<Kernel::MemoryService>();
    auto &port = registers->ports[portNumber];

    // in seiner impl hat er eine ref zu jeder cmd liste
    auto commandList = virtualCommandLists[portNumber];     //wie ist virtualCommandLists? // in der HBA gibt es eine Liste, in der alle command header drin sind?

    //einige Sicherheitssachen
    portLocks[portNumber].acquire();

    if (!port.isActive()) {
        portLocks[portNumber].release();
        return nullptr;
    }
    // finde den richtigen header, an den etwas geschrieben werden kann
    auto slot = findCommandSlot(portNumber);

    if (slot == UINT32_MAX) {
        portLocks[portNumber].release();
        return nullptr;
    }

    //wie funktioniert dma buffer?
    auto dmaBuffer = allocateDmaBuffer(byteCount);
    auto physicalDmaAddress = memoryService.getPhysicalAddress(dmaBuffer);

    //copy von allen wichtigen Werten
    auto commandTable = HbaCommandTable::createCommandTable(byteCount, physicalDmaAddress);
    Util::Address(commandTable->commandFis).copyRange(Util::Address(commandFis), sizeof(HbaCommandTable::commandFis));
    Util::Address(commandTable->atapiCommand).copyRange(Util::Address(atapiCommand), sizeof(HbaCommandTable::atapiCommand));

    // nachdem die command table fertig ist, muss noch der Header der command table richtig mit Werten befüllt werden
    auto &commandHeader = commandList[slot];
    commandHeader.clear();
    commandHeader.physicalRegionDescriptorTableLength = byteCount % BYTES_PER_DESCRIPTOR_ENTRY == 0 ? (byteCount / BYTES_PER_DESCRIPTOR_ENTRY) : (byteCount / BYTES_PER_DESCRIPTOR_ENTRY) + 1;
    commandHeader.commandFisLength = sizeof(FisRegisterHostToDevice) / sizeof(uint32_t);
    commandHeader.commandTableDescriptorBaseAddress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(commandTable));
    commandHeader.atapi = atapiCommand[0] == 0 ? 0 : 1;

    // Issue command
    //falls es irgendwo Probleme gibt
    if (!port.issueCommand(slot)) {
        portLocks[portNumber].release();
        delete reinterpret_cast<uint8_t*>(dmaBuffer);
        delete commandTable;
        return nullptr;
    }

    // am Ende soll wohl alles im dmaBuffer stehen
    // unsicher, ob das so mit dem Typ richtig ist, oder ich da noch was machen muss
    portLocks[portNumber].release();
    delete commandTable;
    return dmaBuffer;
}*/

    //allocate memory into the heap
    pub fn allocate_heap_region(size: u32) -> Vec<u32> {
        let mut output = vec![0; size as usize];
        output
    }
    pub fn allocate_dma_buffer(size: u32) -> Vec<u32>{
        Self::allocate_heap_region(size)
    }

    /*void *AhciController::allocateDmaBuffer(uint32_t size) {
    const auto dmaPages = size % Util::PAGESIZE == 0 ? (size / Util::PAGESIZE) : (size / Util::PAGESIZE) + 1;
    return Kernel::Service::getService<Kernel::MemoryService>().mapIO(dmaPages);
    }*/

    pub fn create_combined_hba_cmd_table(&self, byte_count: u32, physical_dma_buffer : *mut u32) ->combined_HBA_CommandTable{
        let cmd_table = self.create_hba_cmd_table();
        let mut descriptor_count;
        if byte_count / 4096 == 0{
            descriptor_count = (byte_count / 4096) +1;
        }else{
            descriptor_count = byte_count / 4096;
        }
        info!("descriptor count ist {}", descriptor_count);
        let cmd_vec = self.create_cmd_vec(byte_count, physical_dma_buffer, descriptor_count);

        combined_HBA_CommandTable{
            cmd_table,
            physicalRegionDescriptorTable: cmd_vec,
        }
    }
    pub fn create_hba_cmd_table(&self)->HbaCommandTable{
        // füllt nur mit 0 auf, weil das später anders reinkopiert wird
        let output = HbaCommandTable{
            commandFis: [0;64],
            atapiCommand: [0;16],
            reserved: [0;48],
        };
        output
    }
    pub fn create_cmd_vec(&self, byte_count: u32, physical_dma_buffer : *mut u32, descriptor_count: u32) ->Vec<HbaPhysicalRegionDescriptorTableEntry>{
        let mut output : Vec<HbaPhysicalRegionDescriptorTableEntry> = Vec::<HbaPhysicalRegionDescriptorTableEntry>::new();
        unsafe {
            for i in 0..descriptor_count{
                info!("dma_buffer is at {:p}", physical_dma_buffer);
                let remaining_count = byte_count - (i * 4096);
                let mut entry_byte_count = 4096 -1;
                if remaining_count < 4096{
                    entry_byte_count = remaining_count -1;
                }
                let new_entry = HbaPhysicalRegionDescriptorTableEntry{
                    dataBaseAddress: physical_dma_buffer.offset((i * 4096) as isize),
                    dataBaseAddressUpper: null::<u32>().cast_mut(),
                    reserved1: 0,
                    rest: entry_byte_count <<10,
                };
                output.push(new_entry);
            }
        }
        output
    }

    /*AhciController::HbaCommandTable * AhciController::HbaCommandTable::createCommandTable(uint32_t byteCount, void physicalDmaBuffer) {
    #auto &memoryService = Kernel::Service::getService<Kernel::MemoryService>();

    //also dividieren und falls 0, dann ein mehr?
    #auto descriptorCount = byteCount % BYTES_PER_DESCRIPTOR_ENTRY == 0 ? (byteCount / BYTES_PER_DESCRIPTOR_ENTRY) : (byteCount / BYTES_PER_DESCRIPTOR_ENTRY) + 1;

    // hier wird die gesamte größe fürs Mapping berechnet (noch zu tun?)
    auto tableSize = sizeof(commandFis) + sizeof(atapiCommand) + sizeof(reserved) + descriptorCount * sizeof(HbaPhysicalRegionDescriptorTableEntry);
    auto tablePages = tableSize % Util::PAGESIZE == 0 ? (tableSize / Util::PAGESIZE) : (tableSize / Util::PAGESIZE) + 1;

    // an Addr für command table wird das Struct gesetzt (ist das nicht schon so impl?)
    auto commandTable = reinterpret_cast<HbaCommandTable>(memoryService.mapIO(tablePages));
    Util::Address(commandTable).setRange(0, tableSize);

    #for (uint32_t i = 0; i < descriptorCount; i++) {
       # auto &entry = commandTable->physicalRegionDescriptorTable[i];

        #uint32_t remainingBytes = byteCount - (i * BYTES_PER_DESCRIPTOR_ENTRY);
        #entry.dataBaseAddress = reinterpret_cast<uint32_t>(physicalDmaBuffer) + i * BYTES_PER_DESCRIPTOR_ENTRY;
        #entry.dataByteCount = (remainingBytes < BYTES_PER_DESCRIPTOR_ENTRY ? remainingBytes : BYTES_PER_DESCRIPTOR_ENTRY) - 1;
    }
    #return commandTable;
    }*/




}

// Todo:
//Comand Liste anschauen (es werden 31 command slots unterstützt) (es wird kein weiterer gefunden)
//command table mit allen 32 headern versuchen zu allocaten


//prdt mappen und genauer anschauen:
//  das Feld prdt, welches aktuell noch zusammen ist, muss auf 8 begrenzt werden (fertig)

//command table mit Werten befülen / mapping testen (command table ist zu groß und unbestimmt, als dass sie mit Werten gefüllt werden kann. aktuell ist die prdtl = 1)



// Warum wird im HHU OS ein Fehler mit F zugeschrieben, als reset? (angelescu fragen)
// Warum bekomme ich viele Ports mit der selben Adresse? gibt es nur einen Port, oder woran liegt das?
//welche Verträge hat die Uni mit Verlegern? kostenlose Bücher?



//device erkennung impl
//  read from device impl
    // alloc vom dma Speicher machen (fertig)
    // create command table impl (fertig)
        // fragen, ob das region mapping noch gemacht werden muss
    // verstehen, wie der dma buffer den Inhalt bekommt
// verstehen, wie man von read from device in das struct kommt









