use crate::device::ide::IdeDrive;
use crate::memory::frames::alloc;
use crate::memory::nvmem::NfitStructureHeader;
use crate::memory::vma::VmaType;
use crate::memory::{MemorySpace, PAGE_SIZE, ahciController, frames, pages};
use crate::process::scheduler::Scheduler;
use crate::storage::add_block_device;
use crate::storage::block::BlockDevice;
use crate::syscall::sys_time::{sys_get_system_time, wait_ms};
use crate::{pci_bus, process_manager, scheduler};
use alloc::alloc::alloc_zeroed;
use alloc::boxed::Box;
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
use tock_registers::interfaces::ReadWriteable;
use tock_registers::interfaces::Readable;
use tock_registers::interfaces::Writeable;
use tock_registers::register_bitfields;
use tock_registers::registers::{InMemoryRegister, ReadOnly, ReadWrite};
use x86_64::VirtAddr;
use x86_64::structures::paging::frame::PhysFrameRange;
use x86_64::structures::paging::page::PageRange;
use x86_64::structures::paging::{Page, PageTableFlags};

const MASS_STORAGE_DEVICE: BaseClass = 0x01;
const SATA_CONTROLLER: SubClass = 0x06;

//wird verwendet, um die command engine zu starten und zu stoppen
const START: u32 = 1 << 0;
const FIS_RECIVE_ENABLE: u32 = 1 << 4;
const FIS_RECEIVE_RUNNING: u32 = 1 << 14;
const COMMAND_LIST_RUNNING: u32 = 1 << 15;

//command
const ATA_IDENTIFY: u8 = 0xec;
const ATAPI_IDENTIFY: u8 = 0xa1;
const READ_DMA: u8 = 0xc8;
const READ_DMA_EX: u8 = 0x25;
const WRITE_DMA: u8 = 0xca;
const WRITE_DMA_EX: u8 = 0x35;
const ATA_PACKET: u8 = 0xa0;
const ATAPI_READ: u8 = 0xa8;
const ATAPI_READ_CAPACITY: u8 = 0x25;

//sektorgroesse
const SEKTORGROESSE: u32 = 512;

enum BiosHandoffFlags {
    BIOS_OWNED_SEMAPHORE = 1 << 0,
    OS_OWNED_SEMAPHORE = 1 << 1,
    SMI_ON_OWNERSHIP_CHANGE = 1 << 2,
    OS_OWNERSHIP_CHANGE = 1 << 3,
    BIOS_BUSY = 1 << 4,
}

#[derive(Clone, Copy, Debug)]
enum DeviceSignature {
    NONE = 0x00000000,
    ATA = 0x00000101,
    ATAPI = 0xeb140101,
    ENCLOSURE_POWER_MANAGEMENT_BRIDGE = 0xc33c0101,
    PORT_MULTIPLIER = 0x96690101,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TransferMode {
    READ = 0x0,
    WRITE = 0x1,
}

#[allow(warnings)]
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct AhciController {
    hba_regs: u32,
    ports_start: u32,
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HBARegister {
    //passt das so???
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
    reserved: [u8; 116],
    vendorSpecific: [u8; 96],
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
    reserved2: [u32; 10],
    vendorSpecific: [u32; 4],
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaCommandTableHeader {
    // DWORD 0
    dword0: u32, //ReadWrite<u32, D0::Register>,

    // DWORD 1
    physicalRegionDescriptorByteCount: u32,

    // DWORD 2-3
    commandTableDescriptorBaseAddress: u32,
    commandTableDescriptorBaseAddressUpper: u32,

    // DWORD 4-7
    reserved: [u32; 4],
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaPhysicalRegionDescriptorTableEntry {
    dataBaseAddress: u32,
    dataBaseAddressUpper: u32,
    reserved1: u32,
    databytecount_and_interruptOnCompletion: u32,
    //uint32_t dataByteCount: 22;
    //uint32_t reserved2: 9;
    //uint32_t interruptOnCompletion: 1;
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct HbaCommandTable {
    commandFis: [u8; 64],
    atapiCommand: [u8; 16],
    reserved: [u8; 48],
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct DeviceInfo {
    config: u16,            /* lots of obsolete bit flags */
    cyls: u16,              /* obsolete */
    reserved2: u16,         /* special config */
    heads: u16,             /* "physical" heads */
    track_bytes: u16,       /* unformatted bytes per track */
    bytesPerSector: u16,    /* unformatted bytes per sector */
    sectors: u16,           /* "physical" sectors per track */
    vendor0: u16,           /* vendor unique */
    vendor1: u16,           /* vendor unique */
    vendor2: u16,           /* vendor unique */
    serialNumber: [u8; 20], /* 0 = not specified */
    buf_type: u16,
    buf_size: u16,             /* 512 byte increments; 0 = not specified */
    ecc_bytes: u16,            /* for r/w long cmds; 0 = not specified */
    firmwareRevision: [u8; 8], /* 0 = not specified */
    model: [u8; 40],           /* 0 = not specified */
    multi_count: u16,          /* Multiple Count */
    dword_io: u16,             /* 0=not_implemented; 1=implemented */
    capability1: u16,          /* vendor unique */
    capability2: u16,          /* bits 0:DMA 1:LBA 2:IORDYsw 3:IORDYsup word: 50 */
    vendor5: u8,               /* vendor unique */
    tPIO: u8,                  /* 0 = slow, 1 = medium, 2 = fast */
    vendor6: u8,               /* vendor unique */
    tDMA: u8,                  /* 0 = slow, 1 = medium, 2 = fast */
    field_valid: u16,          /* bits 0:cur_ok 1:eide_ok */
    cur_cyls: u16,             /* logical cylinders */
    cur_heads: u16,            /* logical heads word 55 */
    cur_sectors: u16,          /* logical sectors per track */
    cur_capacity0: u16,        /* logical total sectors on drive */
    cur_capacity1: u16,        /* (2 words, misaligned int)     */
    multsect: u8,              /* current multiple sector count */
    multsect_valid: u8,        /* when (bit0==1) multsect is ok */
    lbaCapacity: u32,          /* total number of sectors */
    dma_1word: u16,            /* single-word dma info */
    dma_mword: u16,            /* multiple-word dma info */
    eide_pio_modes: u16,       /* bits 0:mode3 1:mode4 */
    eide_dma_min: u16,         /* min mword dma cycle time (ns) */
    eide_dma_time: u16,        /* recommended mword dma cycle time (ns) */
    eide_pio: u16,             /* min cycle time (ns), no IORDY */
    eide_pio_iordy: u16,       /* min cycle time (ns), with IORDY */
    words69_70: [u16; 2],      /* reserved words 69-70 */
    words71_74: [u16; 4],      /* reserved words 71-74 */
    queue_depth: u16,
    sata_capability: u16,  /* SATA Capabilities word 76 */
    sata_additional: u16,  /* Additional Capabilities */
    sata_supported: u16,   /* SATA Features supported */
    features_enabled: u16, /* SATA features enabled */
    major_rev_num: u16,    /* Major rev number word 80 */
    minor_rev_num: u16,    /* Minor revision number */
    command_set_1: u16,    /* bits 0: Smart, 1: Security, 2: Removable, 3: PM */
    command_set_2: u16,    /* bits 14:Smart Enabled 13:0 zero */
    cfsse: u16,            /* command set-feature supported extensions */
    cfs_enable_1: u16,     /* command set-feature enabled */
    cfs_enable_2: u16,     /* command set-feature enabled */
    csf_default: u16,      /* command set-feature default */
    dma_ultra: u16,
    word89: u16,                 /* reserved (word 89) */
    word90: u16,                 /* reserved (word 90) */
    CurAPMvalues: u16,           /* current APM values */
    word92: u16,                 /* reserved (word 92) */
    comreset: u16,               /* should be cleared to 0 */
    accoustic: u16,              /*  accoustic management */
    min_req_sz: u16,             /* Stream minimum required size */
    transfer_time_dma: u16,      /* Streaming Transfer Time-DMA */
    access_latency: u16,         /* Streaming access latency-DMA & PIO WORD 97*/
    perf_granularity: u32,       /* Streaming performance granularity */
    total_usr_sectors: [u32; 2], /* Total number of user addressable sectors */
    transfer_time_pio: u16,      /* Streaming Transfer time PIO */
    reserved105: u16,            /* Word 105 */
    sector_sz: u16,              /* Physical Sector size / Logical sector size */
    inter_seek_delay: u16,       /* In microseconds */
    words108_116: [u16; 9],      /* Reserved */
    words_per_sector: u32,       /* words per logical sectors */
    supported_settings: u16,     /* continued from words 82-84 */
    command_set_3: u16,          /* continued from words 85-87 */
    words121_126: [u16; 6],      /* reserved words 121-126 */
    word127: u16,                /* reserved (word 127) */
    security_status: u16,        /* device lock function
                                  * 15:9   reserved
                                  * 8   security level 1:max 0:high
                                  * 7:6   reserved
                                  * 5   enhanced erase
                                  * 4   expire
                                  * 3   frozen
                                  * 2   locked
                                  * 1   en/disabled
                                  * 0   capability */
    csfo: u16,               /* current set features options
                              * 15:4   reserved
                              * 3   auto reassign
                              * 2   reverting
                              * 1   read-look-ahead
                              * 0   write cache */
    words130_155: [u16; 26], /* reserved vendor words 130-155 */
    word156: u16,
    words157_159: [u16; 3],   /* reserved vendor words 157-159 */
    cfa: u16,                 /* CFA Power mode 1 */
    words161_175: [u16; 15],  /* Reserved */
    media_serial: [u8; 60],   /* words 176-205 Current Media serial number */
    sct_cmd_transport: u16,   /* SCT Command Transport */
    words207_208: [u16; 2],   /* reserved */
    block_align: u16,         /* Alignement of logical blocks in larger physical blocks */
    WRV_sec_count: u32,       /* Write-Read-Verify sector count mode 3 only */
    verf_sec_count: u32,      /* Verify Sector count mode 2 only */
    nv_cache_capability: u16, /* NV Cache capabilities */
    nv_cache_sz: u16,         /* NV Cache size in logical blocks */
    nv_cache_sz2: u16,        /* NV Cache size in logical blocks */
    rotation_rate: u16,       /* Nominal media rotation rate */
    word218: u16,             /* Reserved  */
    nv_cache_options: u16,    /* NV Cache options */
    words220_221: [u16; 2],   /* reserved */
    transport_major_rev: u16,
    transport_minor_rev: u16,
    words224_233: [u16; 10], /* Reserved */
    min_dwnload_blocks: u16, /* Minimum number of 512 byte units per DOWNLOAD MICROCODE command for mode 03h */
    max_dwnload_blocks: u16, /* Maximum number of 512 byte units per DOWNLOAD MICROCODE command for mode 03h */
    words236_254: [u16; 19], /* Reserved */
    integrity: u16,          /* Cheksum, Signature */
}

#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct FisRegisterHostToDevice {
    // DWORD 0
    typ: u8,
    port_mult_and_cmd_ctrl: u8,

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
pub fn init() {
    //für die Dokumentation
    /*info!("teste pointer");
    unsafe{
        let start = 0x00;
        let u8_ptr = (start as *mut u8).offset(1);               //0x1
        let u16_ptr = (start as *mut u16).offset(1);            //0x2
        let u32_ptr = (start as *mut u32).offset(1);            //0x4
        let u64_ptr = (start as * mut u64).offset(1);           //0x8
        let u128_ptr = (start as *mut u128).offset(1);         //0x10
        let usize_ptr = (start as *mut usize).offset(1);      //0x8
    }*/

    info!("searching the bus for mass storage devices that use sata");
    let mut found_devices = pci_bus().search_by_class(
        MASS_STORAGE_DEVICE as BaseClass,
        SATA_CONTROLLER as SubClass,
    );
    info!("found devices nr: {:?}", found_devices.len());
    info!("achtung vor unwrap");
    let mut device = found_devices.pop().unwrap();
    info!("achtung nach unwrap");
    unsafe {
        let mut ahci_controller = Arc::new(AhciController::new(device));
        info!(
            "der ahci controller hat die hba: {:?}",
            (*ahci_controller).hba_regs
        );
        info!("check, if bios handoff needed");
        ahci_controller.check_bios_handoff();
        info!("check nr of available ports using capabilities");
        let amt_ports = ahci_controller.check_cap_nr_of_ports();
        info!("es werden {} viele ports unterstützt", amt_ports);
        for i in 0..amt_ports {
            ahci_controller.rebase_port(i);
        }
        ahci_controller.test_identify_all_ports();
        ahci_controller.init_all_ports_as_block_devices();

        info!("check if ports have ata");
        ahci_controller.check_ports_for_device();
        info!("check if device has ahci mode enabled");
        ahci_controller.check_ahci_mode_enabled();
        info!("check if device has only ahci mode enabled");
        ahci_controller.check_only_ahci();
        info!("check if 64 bit addresses are supported");
        ahci_controller.check_64_bit_addr_supported();
        info!("check nr of available command slots");
        ahci_controller.check_nr_of_command_slots();

        // hier wird in den Speicher geschrieben/ gelesen

        //ahci_controller.test_read(1, 1);
        //ahci_controller.test_write(1, 1);
        // bei zu hoher Sektorgroesse macht der Speicher nicht mehr mit...
        ahci_controller.benchmark_read(9000, 10);
    }
    //die GHCR sind in Section 3 der Spezifikation zu finden. ich weiß noch nicht, wie man bis dahin kommt
}

#[allow(warnings)]
impl AhciController {
    fn get_cmd_table_header(start: *mut u8) -> *mut HbaCommandTableHeader {
        unsafe { start as *mut HbaCommandTableHeader }
    }

    unsafe fn new(device: &RwLock<EndpointHeader>) -> Self {
        let device_header = device.read();

        // bei base address register (bar5) stehen die wichtigen Daten für die pci capabilities, register, etc.
        let bar5 = device_header.bar(5, &pci_bus().config_space());
        // bei bar4 findet sich ein io port
        let bar4 = device_header.bar(4, &pci_bus().config_space());
        info!("bar with slot one has the following info: {:?}", bar5);
        let bar_io = bar4.unwrap().unwrap_io();
        let bar_mem = bar5.unwrap().unwrap_mem();
        info!("bar io is {:?} and bar mem is {:?}", bar_io, bar_mem);

        let ahci_base_addr = bar_mem.0 as *mut u8;

        //der Controller startet auf den registern und daran hängen die ports

        //map the memory where the control registers are located
        //hier muss der gesamte ahci controller gemappt werden!!, nicht nur die adressen
        Self::map_general(bar_mem.0 as u64, bar_mem.1 as u64, "ahci");

        let mut hba_regs = ahci_base_addr as u32; // *mut HBARegister;
        let mut hba_regs_pointer = ahci_base_addr as *mut HBARegister;

        let ports_start = hba_regs_pointer.offset(1) as u32; // *mut HbaPort;

        AhciController {
            hba_regs,
            ports_start,
        }
    }

    //length is in bytes
    pub unsafe fn map_general(address: u64, length: u64, tag: &str) {
        info!(
            "Found non-volatile memory (Address: [0x{:x}], Length: [{} B])",
            address, length
        );

        info!(
            "length is {} and num_pages is {}",
            length,
            length / PAGE_SIZE as u64
        );
        let process = process_manager()
            .read()
            .kernel_process()
            .expect("Failed to get kernel process");

        // Map non-volatile memory range to kernel address space
        let start_page = pages::page_from_u64(address).expect("address is not page aligned");
        let start_page_frame =
            frames::frame_from_u64(address).expect("address is not page aligned");

        // Allocate virtual memory area for the non-volatile memory
        let vma = process
            .virtual_address_space
            .alloc_vma(
                Some(start_page),
                length / PAGE_SIZE as u64,
                MemorySpace::Kernel,
                VmaType::DeviceMemory,
                tag,
            )
            .expect("alloc_vma failed");

        // Map non-volatile memory to the kernel address space
        process
            .virtual_address_space
            .map_pfr_for_vma(
                &vma,
                PhysFrameRange {
                    start: start_page_frame,
                    end: start_page_frame + (length / PAGE_SIZE as u64),
                },
                PageTableFlags::PRESENT | PageTableFlags::WRITABLE,
            )
            .expect("map_pfr_for_vma failed for NVRAM");
    }

    pub fn general_bit_check(register: u32, bit_position: u8) -> bool {
        let mask = 1 << bit_position;
        return register & mask != 0;
    }

    pub fn general_bitlen_reader(register: u32, bit_position: u8, len: u8) -> u32 {
        let mut mask = 1 << bit_position;
        for i in 0..len {
            mask = mask | 1 << (bit_position + i)
        }
        return (register & mask) >> bit_position;
    }

    pub fn translate_signature(sign: u32) -> DeviceSignature {
        match sign {
            0x00000000 => return DeviceSignature::NONE,
            0x00000101 => return DeviceSignature::ATA,
            0xeb140101 => return DeviceSignature::ATAPI,
            0xc33c0101 => return DeviceSignature::ENCLOSURE_POWER_MANAGEMENT_BRIDGE,
            0x96690101 => return DeviceSignature::PORT_MULTIPLIER,
            _ => {
                info!("value not found");
                return DeviceSignature::NONE;
            }
        }
    }

    pub unsafe fn check_ports_for_device(&self) {
        let amt_port = self.check_cap_nr_of_ports();
        for i in 0..amt_port - 1 {
            let current_port = (self.ports_start as *mut HbaPort).offset(i.try_into().unwrap());
            info!("teste Port mit der Nummer {}", i);
            if Self::check_port_usable(current_port) {
                let signature = (*current_port).signature;
                info!(
                    "the device signature is {:?}",
                    Self::translate_signature(signature)
                );
            }
        }
    }

    pub unsafe fn test_identify_all_ports(&self) {
        let amt_port = self.check_cap_nr_of_ports();
        for i in 0..amt_port - 1 {
            let current_port = (self.ports_start as *mut HbaPort).offset(i.try_into().unwrap());
            info!("teste Port mit der Nummer {}", i);
            if Self::check_port_usable(current_port) {
                Self::test_identify_device_on_port(&self, i);
            }
        }
    }

    pub unsafe fn check_port_usable(port: *mut HbaPort) -> bool {
        let ssts = (*port).sataStatus;
        let ipm = (ssts >> 8) & 0x0F;
        let det = ssts & 0x0F;
        if ipm != 0x01 {
            //0x01 means that the interface of the device is active. only then the device can be accessed
            info!("ERR: interface is not active");
            return false;
        }
        if det != 0x03 {
            //0x03 means that the device is detected and a physical communication is established
            info!("ERR: device is not detected, or physical communication not established");
            return false;
        }
        true
    }

    pub unsafe fn check_ahci_mode_enabled(&self) {
        let ghc = (*(self.hba_regs as *mut HBARegister)).globalHostControl;
        let output = Self::general_bit_check(ghc, 31);
        if output {
            info!("der Controller läuft im ahci modus");
        } else {
            info!("der Controller läuft nicht im ahci modus");
        }
    }

    pub unsafe fn check_only_ahci(&self) {
        let sam = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let output = Self::general_bit_check(sam, 18);
        if output {
            info!("der Controller unterstützt nur ahci");
        } else {
            info!("der Controller unterstützt nicht nur ahci");
        }
    }

    //todo, falls noch nötig
    pub unsafe fn check_bios_handoff(&self) {
        //check if the version is high enough
        info!("enter check bios handoff func");
        let version = (*(self.hba_regs as *mut HBARegister)).version;
        if version >= 0x10200 {
            info!("Version ist hoch genug");
            let ext_cap = (*(self.hba_regs as *mut HBARegister)).extendedHostCapabilities;
            info!("ext_cap sind {}", ext_cap);
            if ext_cap & 1 != 0 {
                info!("BIOS Handoff wird vom Controller unterstützt")
            }
        } else {
            info!("Version ist nicht hoch genug")
        }
        let handoff = (*(self.hba_regs as *mut HBARegister)).biosHandoffControl;
        if handoff == 0 {
            info!("the bios has no control over the hba, so the os can use it");
        }
    }

    pub unsafe fn check_64_bit_addr_supported(&self) {
        let cap = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let output = Self::general_bit_check(cap, 31);
        if output {
            info!("es werden 64 bit adressen unterstützt");
        } else {
            info!("es werden 32 bit adressen unterstützt");
        }
    }

    pub unsafe fn check_cap_nr_of_ports(&self) -> u32 {
        let cap = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let nr_of_ports = Self::general_bitlen_reader(cap, 0, 5);
        info!(
            "laut capabilities werden {} Ports unterstützt.",
            nr_of_ports
        );
        nr_of_ports
    }

    pub unsafe fn check_nr_of_command_slots(&self) -> u32 {
        let cap = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let nr_of_cmds = Self::general_bitlen_reader(cap, 8, 5);
        info!(
            "laut capabilities werden {} Command slots unterstützt.",
            nr_of_cmds
        );
        nr_of_cmds
    }

    pub unsafe fn start_cmd_engine(&self, mut port: *mut HbaPort) {
        while ((*port).command & COMMAND_LIST_RUNNING) > 0 {
            wait_ms(10);
        }
        (*port).command |= (START | FIS_RECIVE_ENABLE);
    }

    pub unsafe fn stop_cmd_engine(&self, mut port: *mut HbaPort) {
        (*port).command &= (START | FIS_RECIVE_ENABLE);
        while ((*port).command & (FIS_RECEIVE_RUNNING | COMMAND_LIST_RUNNING)) > 0 {
            wait_ms(10);
        }
    }

    pub unsafe fn rebase_port(&self, port_nr: u32) {
        let port = (self.ports_start as *mut HbaPort).offset(port_nr.try_into().unwrap());
        info!("port nr {} hat die addr {:?}", port_nr, port);
        if Self::check_port_usable(port) {
            info!("portnr {} bekommt den rebase", port_nr);
            self.stop_cmd_engine(port);
            let allocated = frames::alloc(2);
            let full_addr = allocated.start.start_address().as_u64();
            let lower_addr = full_addr as u32;
            let upper_addr = (full_addr >> 32) as u32;
            (*port).commandListBaseAddress = lower_addr;
            (*port).commandListBaseAddressUpper = upper_addr;
            self.start_cmd_engine(port);
            (*port).sataError = 0x00000000; //0xffffffff;
            (*port).interruptStatus = 0x00000000; //0xffffffff;
            (*port).interruptEnable = 0x00000000;
            info!("rebase of port {} done", port_nr);
        }
    }

    //finden eines freien command headers über den port
    pub unsafe fn find_cmd_slot(&self, mut port: *mut HbaPort) -> i32 {
        let nr_cmd_slots = self.check_nr_of_command_slots();
        let mut slots = (*port).sataActive | (*port).sataError;
        info!("slots ist {:b}", slots);
        for i in 0..nr_cmd_slots {
            if (slots & 1) == 0 {
                info!("slot gefunden an Stelle {}", i);
                return i as i32;
            }
            slots >>= 1;
        }
        info!("kein Slot gefunden!");
        -1
    }

    //brauche ich das überhaupt noch???
    /*pub unsafe fn find_slot_all_ports(&self) -> Option<*mut HbaPort> {
        let amt_port = (*self.hba_regs).portsImplemented;
        for i in 0..amt_port - 1 {
            let current_port = self.ports_start.offset(i.try_into().unwrap());
            if Self::check_port_usable(current_port) {
                if (self.find_cmd_slot(current_port)) != -1 {
                    return Some(current_port);
                }
            }
        }
        None
    }*/

    //heap Speicher alloziieren
    pub unsafe fn allocate_heap_region(size: u32) -> PhysFrameRange {
        let mut frame_count;
        let full_amt = size / 4096;
        let rest = size % 4096;
        info!("full amt is {} and rest ist {}", full_amt, rest);
        if rest != 0 {
            frame_count = full_amt + 1;
        } else {
            frame_count = full_amt;
        }
        info!("try to allocate {} fames", frame_count);
        let mut allocated = frames::alloc(frame_count as usize);
        let pointer: *mut u8 = allocated.start.start_address().as_u64() as *mut u8;

        //schreibe 0 in die ganzen Felder
        pointer.write_bytes(0, allocated.len().try_into().unwrap());
        allocated
    }

    pub unsafe fn identify_device(&self, portnr: u32) -> DeviceInfo {
        let mut command_fis = [0u8; 64];
        let mut atapi_cmd = [0u8; 16];

        //prepare the host to device fis (muss das nicht mehr gesendet werden??)
        // das muss noch in den command fis gelegt werden
        let mut host_to_device_fis = FisRegisterHostToDevice {
            typ: 39,
            port_mult_and_cmd_ctrl: 128,
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

        let port = (self.ports_start as *mut HbaPort).offset(portnr.try_into().unwrap());
        if (*port).signature == 257 {
            //port signature if it is an ata port
            host_to_device_fis.command = ATA_IDENTIFY; //identification code for ata
        } else {
            host_to_device_fis.command = ATAPI_IDENTIFY; //identification code for atapi
        }

        // copy the info from the struct into the memory region
        unsafe {
            let ptr = command_fis.as_mut_ptr();
            let src = &host_to_device_fis as *const FisRegisterHostToDevice as *const u8;
            ptr::copy_nonoverlapping(src, ptr, size_of::<FisRegisterHostToDevice>());
        }

        // hier wird ein DMA Buffer erzeugt
        let mut dma_reg = AhciController::allocate_heap_region(SEKTORGROESSE);
        let dma_reg_addr = dma_reg.start.start_address().as_u64();

        self.read_from_device(portnr, dma_reg_addr, SEKTORGROESSE, command_fis, atapi_cmd);

        let mut output = dma_reg_addr as *mut DeviceInfo;
        info!("free in id device");
        frames::free(dma_reg);
        unsafe { output.read() }
    }

    pub unsafe fn read_from_device(
        &self,
        portnr: u32,
        physical_dma: u64,
        byte_count: u32,
        mut command_fis: [u8; 64],
        atapi_command: [u8; 16],
    ) {
        let mut port = (self.ports_start as *mut HbaPort).offset(portnr.try_into().unwrap());
        let mut command_list_addr = (*port).commandListBaseAddress as u64
            | (((*port).commandListBaseAddressUpper as u64) << 32);
        unsafe {
            // weil ich nur bisher einen cmd_header in der Liste habe, kann ich da direkt reinschreiben
            let mut first_cmd_header = Self::get_cmd_table_header(command_list_addr as *mut u8);
            //die command List besteht aus cmd_table_headern, welche selbst dann auf die command Table verweisen

            if Self::check_port_usable(port) != true {
                info!("ERR: Port is not usable");
            }

            let slot = self.find_cmd_slot(port);
            if slot == -1 {
                info!("ERR: Slot nicht gefunden");
            }

            // hier wird nur die command table gemacht, nicht die command list
            let mut cmd_table = self.create_hba_cmd_table(byte_count, physical_dma);
            cmd_table.commandFis = command_fis.clone();
            cmd_table.atapiCommand = atapi_command.clone();

            info!("byte count in write to device ist {}", byte_count);
            let mut physical_region_descriptor_table_length;
            let full_amt = byte_count / 4096;
            let rest = byte_count % 4096;
            if rest != 0 {
                physical_region_descriptor_table_length = full_amt + 1;
            } else {
                physical_region_descriptor_table_length = full_amt;
            }

            //nachschauen, wie ich auf diese Größen komme
            let mut cmd_fis_len = size_of::<FisRegisterHostToDevice>() / size_of::<u32>();
            let mut atapi = 0; //atapi ist 0 weil id device für ata und atapi geräte universell ist
            if atapi_command[0] != 0 {
                atapi = 1;
            }
            //zerteile die Adresse
            let cmd_table_base_addr: u64 = ptr::from_mut(cmd_table) as u64;
            let upper_cmd_table_base_addr: u32 = (cmd_table_base_addr >> 32) as u32;
            let lower_cmd_table_base_addr = cmd_table_base_addr as u32;

            //Feld first zusammenbauen (atapi, cmd_fis_len und prdt_len)
            // atapi ist 0, weil es ein ata Befehl ist
            // cmd_fis_len ist 5
            //prdt_len ist 1 (weil nur eine prdt benötigt wird)
            let dword0 = (physical_region_descriptor_table_length << 16) as u32
                | (atapi << 5) as u32
                | cmd_fis_len as u32;
            (*first_cmd_header).dword0 = dword0;
            info!("dword0 ist {:b}", dword0);

            (*first_cmd_header).commandTableDescriptorBaseAddressUpper = upper_cmd_table_base_addr;
            (*first_cmd_header).commandTableDescriptorBaseAddress = lower_cmd_table_base_addr;

            let success = (*port).issueCommand(slot as u32);
            if !success {
                info!("ERR: issueCommand hatte einen Fehler")
            }

            //Some(physical_dma)
        }
    }

    pub unsafe fn create_hba_cmd_table(
        &self,
        byte_count: u32,
        physical_dma_buffer: u64,
    ) -> &'static mut HbaCommandTable {
        //berechne, wie viele descriptoren benötigt werden
        let mut descriptor_count;
        info!("byte count ist: {:?}", byte_count);
        info!("berechne descriptor_count: {:?}", byte_count / 4096);
        let full_amt = byte_count / 4096;
        let rest = byte_count % 4096;
        info!("full amt is {} and rest ist {}", full_amt, rest);
        if rest != 0 {
            descriptor_count = full_amt + 1;
        } else {
            descriptor_count = full_amt;
        }

        // füllt nur mit 0 auf, weil das später anders reinkopiert wird
        // alloc frame nötig
        // dann addr weitergeben
        //später ggf mehrere frames nötig
        let mut allocated = frames::alloc(1);
        let pointer: *mut u8 = allocated.start.start_address().as_u64() as *mut u8;

        //schreibe 0 in die ganzen Felder
        pointer.write_bytes(0, 4096);
        let output = pointer as *mut HbaCommandTable;

        if descriptor_count == 1 {
            info!("es reicht ein descriptor");
            //descriptor hängt direkt nach der hba_cmd_table
            let mut descriptor = output.offset(1) as *mut HbaPhysicalRegionDescriptorTableEntry;
            (*descriptor).dataBaseAddress = physical_dma_buffer as u32;
            (*descriptor).dataBaseAddressUpper = (physical_dma_buffer >> 32) as u32;
            (*descriptor).databytecount_and_interruptOnCompletion = byte_count - 1;
            info!("done");
        } else {
            info!("descriptor count ist {:?}", descriptor_count);
            for i in 0..descriptor_count {
                let mut descriptor =
                    output.offset((i + 1) as isize) as *mut HbaPhysicalRegionDescriptorTableEntry;

                (*descriptor).dataBaseAddress = (physical_dma_buffer + (i * 4096) as u64) as u32;
                (*descriptor).dataBaseAddressUpper =
                    ((physical_dma_buffer + (i * 4096) as u64) >> 32) as u32;

                let remaining_bytes = byte_count - i * 4096;
                if remaining_bytes < 4096 {
                    (*descriptor).databytecount_and_interruptOnCompletion = remaining_bytes;
                } else {
                    (*descriptor).databytecount_and_interruptOnCompletion = byte_count - 1;
                }
            }
        }

        output.as_mut().unwrap()
    }

    pub unsafe fn byte_swap(&self, input: *mut u8, len: isize) {
        for i in (0..len).step_by(2) {
            let swap = *input.offset(i);
            *input.offset(i) = *input.offset(i + 1);
            *input.offset(i + 1) = swap;
        }
    }

    pub unsafe fn write_to_device(
        &self,
        portnr: u32,
        physical_dma: u64,
        byte_count: u32,
        mut command_fis: [u8; 64],
        atapi_command: [u8; 16],
    ) -> bool {
        let mut port = (self.ports_start as *mut HbaPort).offset(portnr.try_into().unwrap());
        info!("port in write to device ist {:?}", port);
        let mut command_list_addr = (*port).commandListBaseAddress as u64
            | (((*port).commandListBaseAddressUpper as u64) << 32);

        // weil ich nur bisher einen cmd_header in der Liste habe, kann ich da direkt reinschreiben
        let mut first_cmd_header = Self::get_cmd_table_header(command_list_addr as *mut u8);
        //die command List besteht aus cmd_table_headern, welche selbst dann auf die command Table verweisen
        info!(
            "first_cmd_header in read from device is {:?}",
            first_cmd_header
        );

        if Self::check_port_usable(port) != true {
            info!("ERR: Port is not usable");
            return false;
        }

        let slot = self.find_cmd_slot(port);
        if slot == -1 {
            info!("ERR: Slot nicht gefunden");
            return false;
        }

        let mut cmd_table = self.create_hba_cmd_table(byte_count, physical_dma);
        cmd_table.commandFis = command_fis.clone();
        cmd_table.atapiCommand = atapi_command.clone();
        info!("die cmd_table sieht so aus: {:?}", cmd_table);

        // hier wird alles in den cmd header geschrieben
        info!("byte count in write to device ist {}", byte_count);
        let mut physical_region_descriptor_table_length;
        let full_amt = byte_count / 4096;
        let rest = byte_count % 4096;
        if rest != 0 {
            physical_region_descriptor_table_length = full_amt + 1;
        } else {
            physical_region_descriptor_table_length = full_amt;
        }

        //nachschauen, wie ich auf diese Größen komme
        let mut cmd_fis_len = size_of::<FisRegisterHostToDevice>() / size_of::<u32>();
        let mut atapi = 0; //atapi ist 0 weil id device für ata und atapi geräte universell ist
        if atapi_command[0] != 0 {
            atapi = 1;
        }
        let write = 1;

        //zerteile die Adresse
        let cmd_table_base_addr: u64 = ptr::from_mut(cmd_table) as u64;
        let upper_cmd_table_base_addr: u32 = (cmd_table_base_addr >> 32) as u32;
        let lower_cmd_table_base_addr = cmd_table_base_addr as u32;

        //alles zu dem first zusammenfügen (atapi, cmd_fis_len und prdt_len)
        // atapi ist 0, weil es ein ata Befehl ist
        // cmd_fis_len ist 5
        //prdt_len ist 1 (weil nur eine prdt benötigt wird)
        let dword0 = (physical_region_descriptor_table_length << 16) as u32
            | (atapi << 5) as u32
            | (write << 4) as u32       //es soll geschrieben werden
            | cmd_fis_len as u32;
        (*first_cmd_header).dword0 = dword0;
        info!("dword0 write ist {:b}", dword0);

        (*first_cmd_header).commandTableDescriptorBaseAddressUpper = upper_cmd_table_base_addr;
        (*first_cmd_header).commandTableDescriptorBaseAddress = lower_cmd_table_base_addr;

        let success = (*port).issueCommand(slot as u32);
        if !success {
            info!("ERR: issueCommand hatte einen Fehler");
            return false;
        }

        return true;
    }

    pub unsafe fn performAtaIO(
        &self,
        portnr: u32,
        deviceInfo: &DeviceInfo,
        mode: TransferMode,
        mut buffer_addr: u64,
        start_sector: u64,
        sector_count: u32,
    ) -> bool {
        if start_sector + (sector_count as u64) > deviceInfo.lbaCapacity.try_into().unwrap() {
            info!("ERR: AHCI trys to read/write out of bounds!");
            return false;
        }

        let mut command_fis = [0u8; 64];
        let mut atapi_cmd = [0u8; 16];

        //baue das command fis
        let mut host_to_device_fis = FisRegisterHostToDevice {
            typ: 39,                     //Typ = Host To Device
            port_mult_and_cmd_ctrl: 128, //nur command control ist auf 1
            command: 0,
            featureLow: 1, // HBA mode
            lba0: (start_sector & 0xff) as u8,
            lba1: (start_sector >> 8 & 0xff) as u8,
            lba2: (start_sector >> 16 & 0xff) as u8,
            device: 1 << 6, // LBA mode
            lba3: (start_sector >> 24 & 0xff) as u8,
            lba4: 0,
            lba5: 0,
            featureHigh: 0,
            countLow: (sector_count & 0xff) as u8,
            countHigh: (sector_count >> 8 & 0xff) as u8,
            isochronousCommandCompletion: 0,
            control: 0,
            reserved2: 0,
        };
        if mode == TransferMode::READ {
            host_to_device_fis.command = READ_DMA_EX;

            //copy the struct to the array
            unsafe {
                let ptr = command_fis.as_mut_ptr();
                let src = &host_to_device_fis as *const FisRegisterHostToDevice as *const u8;
                ptr::copy_nonoverlapping(src, ptr, size_of::<FisRegisterHostToDevice>());
            }


            self.read_from_device(
                portnr,
                buffer_addr,
                sector_count * (deviceInfo.bytesPerSector as u32),
                command_fis,
                atapi_cmd,
            );
        } else {
            host_to_device_fis.command = WRITE_DMA_EX;
            //copy the struct to the array
            unsafe {
                let ptr = command_fis.as_mut_ptr();
                let src = &host_to_device_fis as *const FisRegisterHostToDevice as *const u8;
                ptr::copy_nonoverlapping(src, ptr, size_of::<FisRegisterHostToDevice>());
            }

            //schicke die Daten an wrtie_to_device

            let buffer_size = sector_count * (deviceInfo.bytesPerSector as u32);


            //hier wird in den Buffer geschrieben

            let success =
                self.write_to_device(portnr, buffer_addr, buffer_size, command_fis, atapi_cmd);
            // todo hier könnten noch allocs gelöscht werden, kommt erst im cleanup
            return success;
        }

        return true;
    }

    unsafe fn test_identify_device_on_port(&self, portnr: u32) {
        info!("identify device on port {}", portnr);
        let id_device = self.identify_device(portnr);
        info!("id device is {:?}", id_device);

        let mut model = id_device.model.clone();
        self.byte_swap(model.as_mut_ptr(), model.len().try_into().unwrap());
        let model_str = String::from_utf8(Vec::from(model)).unwrap();
        let mut serial_nr = id_device.serialNumber.clone();
        self.byte_swap(serial_nr.as_mut_ptr(), serial_nr.len().try_into().unwrap());
        let serial_str = String::from_utf8(Vec::from(serial_nr)).unwrap();
        let mut firmware_rev = id_device.firmwareRevision.clone();
        self.byte_swap(
            firmware_rev.as_mut_ptr(),
            firmware_rev.len().try_into().unwrap(),
        );
        let firmware_str = String::from_utf8(Vec::from(firmware_rev)).unwrap();

        info!(
            "model ist {}, firmware ist {}, seriennummer ist {}",
            model_str, firmware_str, serial_str
        );
    }

    unsafe fn test_read(&self, portnr: u32, arr_len: u32, id_device: DeviceInfo) -> PhysFrameRange {
        if portnr == 0 {
            info!("Achtung es wird vom Bootimage gelesen!");
        }
        let sector_size = id_device.bytesPerSector;

        let read_bytes: u32 = SEKTORGROESSE * arr_len;
        let single_region = AhciController::allocate_heap_region(read_bytes);
        let single_region_addr = single_region.start.start_address().as_u64();
        self.performAtaIO(
            portnr,
            &id_device,
            TransferMode::READ,
            single_region_addr,
            0,
            arr_len,
        );
        single_region
    }

    unsafe fn test_write_debug(&self, portnr: u32, arr_len: u32) {
        if portnr == 0 {
            info!("Achtung es wird ins Bootimage geschrieben!");
        }
        let read_bytes: u32 = SEKTORGROESSE * arr_len;
        let id_device = self.identify_device(portnr);
        //erzeuge einen neuen buffer
        let write_region = AhciController::allocate_heap_region(read_bytes);
        let write_region_addr =write_region.start.start_address().as_u64();
        //wandel den buffer zum slice um
        let mut write_region_ptr = write_region_addr as *mut u8;
        let mut write_sl = core::slice::from_raw_parts_mut(write_region_ptr, read_bytes as usize);
        //schreibe in den slice:
        for i in 0..write_sl.len() {
            write_sl[i] = 9;
        }

        info!(
            "das zu schreibende array ist (kontrollwert): {:?}",
            write_sl
        );
        //schreibe das array an die Stelle in den Speicher:
        self.performAtaIO(
            portnr,
            &id_device,
            TransferMode::WRITE,
            write_region_addr,
            0,
            arr_len,
        );
        info!("Kontrollwert wird geschrieben");
        let ctrl_single_region = AhciController::allocate_heap_region(read_bytes);
        let ctrl_single_region_addr = ctrl_single_region.start.start_address().as_u64();
        self.performAtaIO(
            portnr,
            &id_device,
            TransferMode::READ,
            ctrl_single_region_addr,
            0,
            arr_len,
        );
        let mut ctrl_region_ptr = ctrl_single_region.start.start_address().as_u64() as *mut u8;
        let mut ctrl_readable_array =
            core::slice::from_raw_parts_mut(ctrl_region_ptr, read_bytes as usize);
        info!("das gelesene kontrollarray ist: {:?}", ctrl_readable_array);
        info!("first free in test_write_debug");
        frames::free(ctrl_single_region);
        info!("second free in test_write_debug");
        frames::free(write_region);
    }

    unsafe fn test_write(
        &self,
        portnr: u32,
        arr_len: u32,
        nr_to_write: u8,
        id_device: DeviceInfo,
    ) -> isize {
        if portnr == 0 {
            info!("Achtung es wird ins Bootimage geschrieben!");
        }
        let read_bytes: u32 = SEKTORGROESSE * arr_len;
        //erzeuge einen neuen buffer
        let write_region = AhciController::allocate_heap_region(read_bytes);
        let write_region_addr =write_region.start.start_address().as_u64();
        //wandel den buffer zum slice um
        let mut write_region_ptr = write_region_addr as *mut u8;
        let mut write_sl =
            core::slice::from_raw_parts_mut(write_region_ptr, read_bytes as usize);
        //schreibe in den slice:
        for i in 0..write_sl.len() {
            write_sl[i] = 6;
        }
        //schreibe das array an die Stelle in den Speicher:
        //starte den Timer
        let start_time = sys_get_system_time();
        self.performAtaIO(
            portnr,
            &id_device,
            TransferMode::WRITE,
            write_region_addr,
            0,
            arr_len,
        );
        let end_time = sys_get_system_time();
        let mut read_time = end_time - start_time;
        info!("free in test_write");
        frames::free(write_region);
        read_time
    }

    //diese Funktionen soll dann auch von Block Device ausgeführt werden
    unsafe fn read(
        &self,
        sector: u64,
        count: usize,
        buffer: &mut [u8],
        portnr: u32,
        id_device: DeviceInfo,
    ) -> usize {
        //sector ist der Startsektor
        //count ist die Anzahl der Sektoren
        // in buffer soll reingeschrieben werden
        //output ist die Anzahl an Sektoren
        let sector_size = id_device.bytesPerSector;

        let read_bytes: u32 = SEKTORGROESSE * count as u32;
        let region_buffer = AhciController::allocate_heap_region(read_bytes);
        let region_buffer_addr = region_buffer.start.start_address().as_u64();
        self.performAtaIO(
            portnr,
            &id_device,
            TransferMode::READ,
            region_buffer_addr,
            sector,
            count as u32,
        );

        //kopiere in den output
        // könnte funktionieren
        let mut region_ptr = region_buffer.start.start_address().as_u64() as *mut u8;
        ptr::copy_nonoverlapping(region_ptr, buffer.as_mut_ptr(), read_bytes as usize);

        // hier müsste noch ein free gemacht werden
        info!("free in read");
        frames::free(region_buffer);

        return count;
    }

    unsafe fn write(
        &self,
        sector: u64,
        count: usize,
        buffer: &[u8],
        portnr: u32,
        id_device: DeviceInfo,
    ) -> usize {
        //sector ist der Startsektor
        //count ist die Anzahl der Sektoren
        // in buffer soll reingeschrieben werden
        //output ist die Anzahl an Sektoren
        let read_bytes: u32 = SEKTORGROESSE * count as u32;

        //erstelle eine PhysFrameRange für ataIO
        let region = AhciController::allocate_heap_region(read_bytes);
        let region_addr = region.start.start_address().as_u64();

        //kopiere den Buffer in die Region
        let mut region_ptr = region_addr as *mut u8;
        ptr::copy_nonoverlapping(buffer.as_ptr(), region_ptr, read_bytes as usize);

        // reiche alles an ataIO weiter
        let success = self.performAtaIO(
            portnr,
            &id_device,
            TransferMode::WRITE,
            region_addr,
            sector,
            count as u32,
        );
        if !success {
            return 0;
        }

        //gib den Speicher wieder frei
        info!("free in write");
        frames::free(region);

        return count;
    }

    pub unsafe fn init_all_ports_as_block_devices(&self) {
        let amt_port = self.check_cap_nr_of_ports();
        for i in 0..amt_port - 1 {
            let current_port = (self.ports_start as *mut HbaPort).offset(i.try_into().unwrap());
            info!("init Port {} as block device", i);
            if Self::check_port_usable(current_port) {
                let ahci_drive = Arc::new(AHCIDrive::new(Arc::new(self.clone()), i));
                add_block_device("ata", ahci_drive);
            }
        }
    }

    // hier beginnen die Benchmarks

    // vielleicht die Device Info weitergeben?
    // first scenario of Benchmarking: read a lot of sectors in a sequence

    pub unsafe fn benchmark_check_single_read(
        &self,
        sector_count: u32,
        correct_arr: &[u8],
        id_device: DeviceInfo,
    ) -> isize {
        // test, if the read amt of sectors is correct.
        //times only during the reading process and returns the time in ms
        // if the read sectors does not fit with the correct array, it returns -1

        //start timer:
        let start_time = sys_get_system_time();

        let read_sectors = self.test_read(1, sector_count, id_device);

        let end_time = sys_get_system_time();

        let mut read_time = end_time - start_time;

        let read_bytes = SEKTORGROESSE * sector_count;
        let mut region_ptr = read_sectors.start.start_address().as_u64() as *mut u8;
        let array = core::slice::from_raw_parts_mut(region_ptr, read_bytes as usize);

        //check if the read_sectors are correct
        info!("read sectors sind: {:?}", array.len());
        let mut equal = true;
        for i in 0..array.len(){
            if array[i] != correct_arr[i]{
                info!("array an stelle {} ist {}, und korrect wäre {}", i, array[i], correct_arr[i]);
                equal = false;
                break;
                // problem: ab 860486 wird nur 255 ausgelesen. was stimmt da mit der Platte nicht??
            }
        }

        if equal {
            //irgendwie wieder die read sectors herausbekommen und dann mit free arbeiten
            info!("free in benchmark_check_single_read, in if yes");
            frames::free(read_sectors);
            read_time
        } else {
            info!("free in benchmark_check_single_read, in if no");
            frames::free(read_sectors);
            -1 as isize
        }
    }

    pub unsafe fn benchmark_read(&self, sector_count: u32, repetitions: u32) {
        //repetitions should be a multiple of 10
        // all benchmarks on hdd.img
        info!(
            "start read benchmark, with {} sectors in a sequence and {} repetitions",
            sector_count, repetitions
        );
        let id_device = self.identify_device(1);
        let correct_arr = self.test_read(1, sector_count, id_device);
        let read_bytes = SEKTORGROESSE * sector_count;
        let mut region_ptr = correct_arr.start.start_address().as_u64() as *mut u8;
        let array = core::slice::from_raw_parts_mut(region_ptr, read_bytes as usize);

        let mut full_time_ms = 0;
        let mut amt_success = 0;

        for i in 0..repetitions {
            let single_result = self.benchmark_check_single_read(sector_count, array, id_device);
            if single_result != -1 {
                full_time_ms += single_result;
                amt_success += 1;
            }
        }
        info!("free in benchmark_read, könnte das Problem sein");
        //frames::free(correct_arr);
        info!(
            "finished read benchmark, with {} sectors in a sequence and {} repetitions",
            sector_count, repetitions
        );
        info!(
            "managed to read {} of {} times successfully with a complete time of {} ms",
            amt_success, repetitions, full_time_ms
        );
    }

    pub unsafe fn benchmark_one_write(&self, sector_count: u32, nr_to_write: u8) {
        let id_device = self.identify_device(1);
        let read_bytes: u32 = SEKTORGROESSE * sector_count;
        let write_region = AhciController::allocate_heap_region(read_bytes);
        //wandel den buffer zum slice um
        let mut write_region_ptr = write_region.start.start_address().as_u64() as *mut u8;
        let mut correct = core::slice::from_raw_parts_mut(write_region_ptr, read_bytes as usize);
        //schreibe in die slice:
        for i in 0..correct.len() {
            correct[i] = nr_to_write;
        }
        info!("free in benchmark_one_write");
        frames::free(write_region);

        todo!("hier muss noch weiter gearbeitet werden!!!")
    }

    pub unsafe fn benchmark_write(&self, sector_count: u32, repetitions: u32) {
        //repetitions should be a multiple of 10
        // all benchmarks on hdd.img
    }

    //second scenario: read and write sectors at random spots
}

#[allow(warnings)]
impl HbaPort {
    pub fn issueCommand(&mut self, slot: u32) -> bool {
        // Wait while device is busy
        const COMMAND_TIMEOUT: isize = 10000;
        const BUSY: u32 = 128;
        const DATA_TRANSFER_REQUESTED: u32 = 8;
        const TASK_FILE_ERROR: u32 = 1 << 30;
        let mut timeout = sys_get_system_time() + COMMAND_TIMEOUT;

        while (self.taskFileData & (BUSY | DATA_TRANSFER_REQUESTED)) > 0 {
            if (sys_get_system_time() >= timeout) {
                info!("system timeout 1");
                return false;
            }
            scheduler().switch_thread_no_interrupt();
        }

        // Issue command
        self.commandIssue = 1 << slot;

        // Wait for command completion
        timeout = sys_get_system_time() + COMMAND_TIMEOUT;
        while true {
            if ((self.commandIssue & (1 << slot)) == 0) {
                break;
            }

            if (self.interruptStatus & TASK_FILE_ERROR) > 0 {
                info!("interrupt status and task file error");
                return false;
            }

            if (sys_get_system_time() >= timeout) {
                info!("system timeout 2");
                return false;
            }
            scheduler().switch_thread_no_interrupt();
        }
        true
    }
}

pub struct AHCIDrive {
    controller: Arc<AhciController>, // können mehrere Drives sich einen AHCI Controller nehmen? im ahci controller stehen ja nur Adressen
    info: DeviceInfo,
    portnr: u32,
}

impl AHCIDrive {
    fn new(controller: Arc<AhciController>, portnr: u32) -> Self {
        let info = unsafe { controller.identify_device(portnr) };
        Self {
            controller,
            info,
            portnr,
        }
    }
}

impl BlockDevice for AHCIDrive {
    fn read(&self, sector: u64, count: usize, buffer: &mut [u8]) -> usize {
        unsafe {
            self.controller
                .read(sector, count, buffer, self.portnr, self.info);
            count
        }
    }

    fn write(&self, sector: u64, count: usize, buffer: &[u8]) -> usize {
        unsafe {
            self.controller
                .write(sector, count, buffer, self.portnr, self.info);
            count
        }
    }

    fn sector_count(&self) -> u64 {
        self.info.lbaCapacity as u64
    }

    fn sector_size(&self) -> u16 {
        self.info.bytesPerSector
    }
}

/*

    //in der init müssen noch die Block Devices richtig gemacht werden:
pub fn init() {
    let devices = pci_bus().search_by_class(0x01, 0x01);
    for device in devices {
        let device_id = device.read().header().id(pci_bus().config_space());
        info!("Found IDE controller [{}:{}]", device_id.0, device_id.1);

        let ide_controller = Arc::new(IdeController::new(device));
        IdeController::plugin(Arc::clone(&ide_controller));

        let found_drives = ide_controller.init_drives();
        for drive in found_drives.iter() {
            let block_device = Arc::new(IdeDrive::new(Arc::clone(&ide_controller), *drive));
            add_block_device("ata", block_device);
        }
    }
}

*/

// Todo:
//Comand Liste anschauen (es werden 31 command slots unterstützt) (es wird kein weiterer gefunden)
//command table mit allen 32 headern versuchen zu allocaten

// Fehler werden mit f zu geschrieben, weil das -1 repräsentiert
// Warum bekomme ich viele Ports mit der selben Adresse? gibt es nur einen Port, oder woran liegt das?  (aktuell existiert ein Port)

//hier die impl für Block Device
// finde die richtige Darstellungsweise, soll ich ein neues struct erstellen?

// Idee: ich mache das Struct so wie im Drive, dann wird innerhalb des Drive nur read, write, info zeug so gemacht. alles andere ist dann in der
// impl des ahci controllers
