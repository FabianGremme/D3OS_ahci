use crate::device::ide::IdeDrive;
use crate::memory::frames::alloc;
use crate::memory::nvmem::NfitStructureHeader;
use crate::memory::vma::VmaType;
use crate::memory::{MemorySpace, PAGE_SIZE, ahciController, frames, pages};
use crate::process::scheduler::Scheduler;
use crate::storage::add_block_device;
use crate::storage::block::BlockDevice;
use crate::syscall::sys_time::{sys_get_system_time, sys_get_system_time_ns, wait_ms};
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
use rand::RngCore;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use spin::RwLock;
use tock_registers::interfaces::ReadWriteable;
use tock_registers::interfaces::Readable;
use tock_registers::interfaces::Writeable;
use tock_registers::register_bitfields;
use tock_registers::registers::{InMemoryRegister, ReadOnly, ReadWrite};
use x86_64::VirtAddr;
use x86_64::instructions::port;
use x86_64::structures::paging::frame::PhysFrameRange;
use x86_64::structures::paging::page::PageRange;
use x86_64::structures::paging::{Page, PageTableFlags};

//pci device numbers
const MASS_STORAGE_DEVICE: BaseClass = 0x01;
const SATA_CONTROLLER: SubClass = 0x06;

//command engine
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

//hardcoded necessary limits for prdt
const SEKTORGROESSE: u32 = 512;
const SEKTORZAHL: usize = 8 * 8 * 1024;

//different sizes of the descriptors for testing purposes
const SMALLDESCRIPTOR: u32 = 4 * 1024;
const BIGDESCRIPTOR: u32 = 4 * 1024 * 1024;

//flags for detailed bios handoff
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

//the main controller addr
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
    dword0: u32,

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
    //representation of the field databytecount_and_interruptOnCompletion:
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
    //representation of the field port_mult_and_cmd_ctrl:
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
    info!("searching the bus for mass storage devices that use sata");
    let mut found_devices = pci_bus().search_by_class(
        MASS_STORAGE_DEVICE as BaseClass,
        SATA_CONTROLLER as SubClass,
    );
    info!("found devices nr: {:?}", found_devices.len());
    let mut device = found_devices[0];
    unsafe {
        let mut ahci_controller = Arc::new(AhciController::new(device));
        info!("ahci controller has hba: {:?}", (*ahci_controller).hba_regs);
        info!("check, if bios handoff needed");
        ahci_controller.check_bios_handoff();
        info!("check nr of available ports using capabilities");
        let amt_ports = ahci_controller.check_cap_nr_of_ports();
        info!("supporting {} ports", amt_ports);
        for i in 0..amt_ports {
            ahci_controller.rebase_port(i);
        }
        info!("identify all ports");
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

        //small experiment to present reading and writing mechanics
        info!("start experimente");
        let portnr = 1;
        let test_add = 0;
        let sector_count = SEKTORZAHL + test_add;
        let id_device1 = ahci_controller.identify_device(portnr);
        info!("test write");
        ahci_controller.test_write(portnr, 0, sector_count, 6, &id_device1);
        info!("test read");

        //Experiment für test_read
        let read_bytes: u64 = (SEKTORGROESSE * sector_count as u32) as u64;
        info!("bytes to read: {}", read_bytes);
        let single_region = AhciController::allocate_heap_region(read_bytes);
        let single_region_ptr = single_region.start.start_address().as_u64() as *mut u8;
        let buffer = core::slice::from_raw_parts_mut(single_region_ptr, read_bytes as usize);
        let correct_read_bytes =
            ahci_controller.test_read(0, sector_count as usize, buffer, portnr, &id_device1);
        let mut success = true;
        let mut bad_counter = 0;
        for i in 0..buffer.len() {
            if buffer[i] != 6 {
                info!("found: {}, on position {}", &buffer[i], i);
                success = false;
                bad_counter += 1;
                break;
            }
        }
        info!(
            "the experiment was {}, with {} wrong bytes",
            success, bad_counter
        );

        // alle sequenziellen Benchmarks
        /*ahci_controller.benchmark_read(200, 100, 0);
        ahci_controller.benchmark_write(200, 100, 0);

        ahci_controller.benchmark_read(1024 *2, 100, 0);
        ahci_controller.benchmark_write(1024 * 2, 100, 0);

        ahci_controller.benchmark_read(1024 * 10, 100, 0);
        ahci_controller.benchmark_write(1024 * 10, 100, 0);

        ahci_controller.benchmark_read(1024 * 20, 100, 0);
        ahci_controller.benchmark_write(1024 * 20, 100, 0);

        ahci_controller.benchmark_read(1024 * 40, 100, 0);
        ahci_controller.benchmark_write(1024 * 40, 100, 0);

        ahci_controller.benchmark_read(1024 * 100, 100, 0);
        ahci_controller.benchmark_write(1024 * 100, 100, 0);

        ahci_controller.benchmark_read(1024 * 1024 * 1, 100, 0);
        ahci_controller.benchmark_write(1024 * 1024 * 1, 100, 0);*/

        /*let mut read_times: Vec<isize> = Vec::new();
        let mut write_times: Vec<isize> = Vec::new();

        for i in 0..100 {
            //w100k.push(ahci_controller.benchmark_random_read(200, 0));
            read_times.push(ahci_controller.benchmark_random_read(2048*5, 0));
            write_times.push(ahci_controller.benchmark_random_read(2048*5, 0));
            /*r1m.push(ahci_controller.benchmark_random_write(2048, 0));
            w5m.push(ahci_controller.benchmark_random_read(10240, 0));
            r5m.push(ahci_controller.benchmark_random_write(10240, 0));
            w10m.push(ahci_controller.benchmark_random_read(20480, 0));
            r10m.push(ahci_controller.benchmark_random_write(20480, 0));
            w20m.push(ahci_controller.benchmark_random_read(40960, 0));
            r20m.push(ahci_controller.benchmark_random_write(40960, 0));
            w50m.push(ahci_controller.benchmark_random_read(102400, 0));
            r50m.push(ahci_controller.benchmark_random_write(102400, 0));*/
        }

        let q1 = &read_times[0..10];
        let q2 = &read_times[10..20];
        let q3 = &read_times[20..30];
        let q4 = &read_times[30..40];
        let q5 = &read_times[40..50];
        let q6 = &read_times[50..60];
        let q7 = &read_times[60..70];
        let q8 = &read_times[70..80];
        let q9 = &read_times[80..90];
        let q10 = &read_times[90..100];
        info!(
            "times of the read benchmarks: \n{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}",
            q1, q2, q3, q4, q5, q6, q7, q8, q9, q10
        );

        let wq1 = &write_times[0..10];
        let wq2 = &write_times[10..20];
        let wq3 = &write_times[20..30];
        let wq4 = &write_times[30..40];
        let wq5 = &write_times[40..50];
        let wq6 = &write_times[50..60];
        let wq7 = &write_times[60..70];
        let wq8 = &write_times[70..80];
        let wq9 = &write_times[80..90];
        let wq10 = &write_times[90..100];
        info!(
            "times of the write benchmarks: \n{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}",
            wq1, wq2, wq3, wq4, wq5, wq6, wq7, wq8, wq9, wq10
        );*/
    }
}

#[allow(warnings)]
impl AhciController {
    //converts the HBA Command Table Header address to the struct
    fn get_cmd_table_header(start: *mut u8) -> *mut HbaCommandTableHeader {
        unsafe { start as *mut HbaCommandTableHeader }
    }

    //transforming a found pci endpoint header into the ahci controller struct
    unsafe fn new(device: &RwLock<EndpointHeader>) -> Self {
        let device_header = device.read();

        // base address register 5 (bar5) has the needed info like pci capabilities, registers, etc.
        let bar5 = device_header.bar(5, &pci_bus().config_space());
        // bar4 is a not needed io port
        let bar4 = device_header.bar(4, &pci_bus().config_space());
        info!("bar5 has the following info: {:?}", bar5);
        let bar_io = bar4.unwrap().unwrap_io();
        let bar_mem = bar5.unwrap().unwrap_mem();
        info!("bar io is {:?} and bar mem is {:?}", bar_io, bar_mem);

        let ahci_base_addr = bar_mem.0 as *mut u8;

        //allocating and mapping the space for the whole ahci controller with its registers
        //not including prdt or command headers
        Self::map_general(bar_mem.0 as u64, bar_mem.1 as u64, "ahci");

        let mut hba_regs = ahci_base_addr as u32;
        let mut hba_regs_pointer = ahci_base_addr as *mut HBARegister;
        // the port struct is directly after the registers
        let ports_start = hba_regs_pointer.offset(1) as u32;

        AhciController {
            hba_regs,
            ports_start,
        }
    }

    /*
    allocating and mapping given addresses with their length and their tag
    length is in bytes
     */
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

    //check if a single bit in a register is set
    pub fn general_bit_check(register: u32, bit_position: u8) -> bool {
        let mask = 1 << bit_position;
        return register & mask != 0;
    }

    //check multiple coherent bits in a register
    pub fn general_bitlen_reader(register: u32, bit_position: u8, len: u8) -> u32 {
        let mut mask = 1 << bit_position;
        for i in 0..len {
            mask = mask | 1 << (bit_position + i)
        }
        return (register & mask) >> bit_position;
    }

    //translate the device signature into its enum
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

    /*
    check if all ports are ready for interaction
    if the port is ready its device signature is returned
    */
    pub unsafe fn check_ports_for_device(&self) {
        let amt_port = self.check_cap_nr_of_ports();
        for i in 0..amt_port - 1 {
            let current_port = (self.ports_start as *mut HbaPort).offset(i.try_into().unwrap());
            info!("test port with number {}", i);
            if Self::check_port_usable(current_port) {
                let signature = (*current_port).signature;
                info!(
                    "the device signature is {:?}",
                    Self::translate_signature(signature)
                );
            }
        }
    }

    //do the identify device on all ports
    pub unsafe fn test_identify_all_ports(&self) {
        let amt_port = self.check_cap_nr_of_ports();
        for i in 0..amt_port - 1 {
            let current_port = (self.ports_start as *mut HbaPort).offset(i.try_into().unwrap());
            info!("try to identify device on port number {}", i);
            if Self::check_port_usable(current_port) {
                Self::test_identify_device_on_port(&self, i);
            }
        }
    }

    //check if the communication with the port is established
    pub unsafe fn check_port_usable(port: *mut HbaPort) -> bool {
        let ssts = (*port).sataStatus;
        let ipm = (ssts >> 8) & 0x0F;
        let det = ssts & 0x0F;
        if ipm != 0x01 {
            //0x01 means that the interface of the device is active. only then the device can be accessed
            info!("ERR: interface is not active!");
            return false;
        }
        if det != 0x03 {
            //0x03 means that the device is detected and a physical communication is established
            info!("ERR: device is not detected, or physical communication not established!");
            return false;
        }
        true
    }

    //check if the controller runs in the ahci mode
    pub unsafe fn check_ahci_mode_enabled(&self) {
        let ghc = (*(self.hba_regs as *mut HBARegister)).globalHostControl;
        let output = Self::general_bit_check(ghc, 31);
        if output {
            info!("controller runs in ahci mode");
        } else {
            info!("controller does not run in ahci mode");
        }
    }
    //check if the controller only supports the ahci mode via capabilities
    pub unsafe fn check_only_ahci(&self) {
        let sam = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let output = Self::general_bit_check(sam, 18);
        if output {
            info!("controller supports only ahci mode");
        } else {
            info!("controller supports more than ahci mode");
        }
    }

    // check if a bios handoff is needed
    // neither in qemu nor on real hardware extra steps are necessary
    pub unsafe fn check_bios_handoff(&self) {
        //check if the version is high enough
        info!("enter check bios handoff func");
        let version = (*(self.hba_regs as *mut HBARegister)).version;
        if version >= 0x10200 {
            info!("version is high enough");
            let ext_cap = (*(self.hba_regs as *mut HBARegister)).extendedHostCapabilities;
            info!("ext_cap: {}", ext_cap);
            if ext_cap & 1 != 0 {
                info!("BIOS handoff supported")
            }
        } else {
            info!("version not high enough")
        }
        let handoff = (*(self.hba_regs as *mut HBARegister)).biosHandoffControl;
        if handoff == 0 {
            info!("the bios has no control over the hba, so the os can use it");
        }
    }

    //check if the system supports the usage of 64 bit addresses
    pub unsafe fn check_64_bit_addr_supported(&self) {
        let cap = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let output = Self::general_bit_check(cap, 31);
        if output {
            info!("64 bit addresses supported");
        } else {
            info!("32 bit addresses supported");
        }
    }

    // check the number of ports supported via capabilities
    pub unsafe fn check_cap_nr_of_ports(&self) -> u32 {
        let cap = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let nr_of_ports = Self::general_bitlen_reader(cap, 0, 5);
        nr_of_ports
    }

    // check the number of command slots supported via capabilities
    pub unsafe fn check_nr_of_command_slots(&self) -> u32 {
        let cap = (*(self.hba_regs as *mut HBARegister)).hostCapabilities;
        let nr_of_cmds = Self::general_bitlen_reader(cap, 8, 5);
        nr_of_cmds
    }

    // start the command engine
    // necessary for the port rebase
    pub unsafe fn start_cmd_engine(&self, mut port: *mut HbaPort) {
        while ((*port).command & COMMAND_LIST_RUNNING) > 0 {
            wait_ms(10);
        }
        (*port).command |= (START | FIS_RECIVE_ENABLE);
    }

    // stop the command engine
    // necessary for the port rebase
    pub unsafe fn stop_cmd_engine(&self, mut port: *mut HbaPort) {
        (*port).command &= (START | FIS_RECIVE_ENABLE);
        while ((*port).command & (FIS_RECEIVE_RUNNING | COMMAND_LIST_RUNNING)) > 0 {
            wait_ms(10);
        }
    }

    // rebase of the port
    pub unsafe fn rebase_port(&self, port_nr: u32) {
        let port = (self.ports_start as *mut HbaPort).offset(port_nr.try_into().unwrap());
        info!("port {} has address {:?}", port_nr, port);
        if Self::check_port_usable(port) {
            info!("do rebase of port {}", port_nr);
            self.stop_cmd_engine(port);

            // allocate space for the port components
            let allocated = frames::alloc(2);
            let full_addr = allocated.start.start_address().as_u64();
            let lower_addr = full_addr as u32;
            let upper_addr = (full_addr >> 32) as u32;

            //splitting the 64 bit address to two 32 bit addresses if necessary
            (*port).commandListBaseAddress = lower_addr;
            (*port).commandListBaseAddressUpper = upper_addr;
            self.start_cmd_engine(port);

            // reset all the error bits
            (*port).sataError = 0xffffffff;
            (*port).interruptStatus = 0xffffffff;
            (*port).interruptEnable = 0x00000000;
            info!("rebase of port {} done", port_nr);
        }
    }

    //find a command slot on a given port
    pub unsafe fn find_cmd_slot(&self, mut port: *mut HbaPort) -> i32 {
        let nr_cmd_slots = self.check_nr_of_command_slots();
        let mut slots = (*port).sataActive | (*port).sataError;
        for i in 0..nr_cmd_slots {
            if (slots & 1) == 0 {
                return i as i32;
            }
            slots >>= 1;
        }
        info!("ERR: No free slot found!");
        -1
    }

    //allocate memory that does not need to be mapped
    pub unsafe fn allocate_heap_region(size: u64) -> PhysFrameRange {
        let mut frame_count;
        let full_amt = size / 4096;
        let rest = size % 4096;
        if rest != 0 {
            frame_count = full_amt + 1;
        } else {
            frame_count = full_amt;
        }
        let mut allocated = frames::alloc(frame_count as usize);
        let pointer: *mut u8 = allocated.start.start_address().as_u64() as *mut u8;

        allocated
    }
    /*
    identify the device with its given port number
    the device info struct contains all info about the device
     */

    pub unsafe fn identify_device(&self, portnr: u32) -> DeviceInfo {
        let mut command_fis = [0u8; 64];
        let mut atapi_cmd = [0u8; 16];

        //prepare the host to device fis
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

        // get the right ata command for the port signature
        let port = (self.ports_start as *mut HbaPort).offset(portnr.try_into().unwrap());
        if (*port).signature == 257 {
            host_to_device_fis.command = ATA_IDENTIFY; //identification code for ata
        } else {
            host_to_device_fis.command = ATAPI_IDENTIFY; //identification code for atapi
        }

        // copy the fis struct into the memory region
        unsafe {
            let ptr = command_fis.as_mut_ptr();
            let src = &host_to_device_fis as *const FisRegisterHostToDevice as *const u8;
            ptr::copy_nonoverlapping(src, ptr, size_of::<FisRegisterHostToDevice>());
        }

        // create a dma buffer in which the results can be written
        let mut dma_reg = AhciController::allocate_heap_region(SEKTORGROESSE as u64);
        let dma_reg_addr = dma_reg.start.start_address().as_u64();

        // execute the command
        self.read_from_device(portnr, dma_reg_addr, SEKTORGROESSE, command_fis, atapi_cmd);

        // convert the output into the right form
        let mut output = dma_reg_addr as *mut DeviceInfo;
        unsafe { output.read() }
    }

    /*
    create the command table with its descriptors for read and write commands
    if change of the descriptor size is necessary, also change the values in the read, write test_read and test_write function
    a smaller descriptor size is not recommended

    byte_count is the amount of bytes that need to be processed
    physical_dma_buffer is the address of the dma region where the results should be written into
    prdt_start_address is the start address of the memory space, allocated for the command table
    descriptor_count is the number of descriptors needed
     */
    pub unsafe fn create_hba_cmd_table(
        &self,
        byte_count: u32,
        physical_dma_buffer: u64,
        prdt_start_addr: u64,
        descriptor_count: u32,
    ) -> *mut HbaCommandTable {
        //convert the command table from address into struct
        let pointer: *mut u8 = prdt_start_addr as *mut u8;
        let output = pointer as *mut HbaCommandTable;

        //for small commands only one descriptor is needed
        if descriptor_count == 1 {
            //writing all the data into the descriptor
            let mut descriptor = output.offset(1) as *mut HbaPhysicalRegionDescriptorTableEntry;
            //splitting the 64 bit base address into two 32 bit addresses
            (*descriptor).dataBaseAddress = physical_dma_buffer as u32;
            (*descriptor).dataBaseAddressUpper = (physical_dma_buffer >> 32) as u32;
            (*descriptor).databytecount_and_interruptOnCompletion = byte_count - 1;
        } else {
            let mut table =
                output.offset((1) as isize) as *mut HbaPhysicalRegionDescriptorTableEntry;
            //calculate the position of all descriptors and write the date inside
            for i in 0..descriptor_count {
                let descriptor_size = BIGDESCRIPTOR;
                let mut descriptor = table.offset(i as isize);
                let new_addr = physical_dma_buffer + (i * descriptor_size) as u64;
                //splitting the 64 bit base address into two 32 bit addresses
                (*descriptor).dataBaseAddress = new_addr as u32;
                (*descriptor).dataBaseAddressUpper = (new_addr >> 32) as u32;

                let remaining_bytes = byte_count - (i * descriptor_size);
                // all descriptors are equal in size except of the last one
                // the last descriptor may be smaller
                if remaining_bytes < descriptor_size {
                    (*descriptor).databytecount_and_interruptOnCompletion = remaining_bytes - 1;
                } else {
                    (*descriptor).databytecount_and_interruptOnCompletion = descriptor_size - 1;
                }
            }
        }
        output
    }

    /*
    swap two adjacent bytes
    needed for text fields in the identify device function
    badc converts to abcd
    */
    pub unsafe fn byte_swap(&self, input: *mut u8, len: isize) {
        for i in (0..len).step_by(2) {
            let swap = *input.offset(i);
            *input.offset(i) = *input.offset(i + 1);
            *input.offset(i + 1) = swap;
        }
    }

    /*
    performs a read access on a given port

    portnr is the number of the port on which the access needs to take place
    physical_dma is the address of the dma region where the results should be written into
    byte_count is the amount of bytes that need to be processed
    command_fis is the command structure that needs to be processed
    atapi_command is a possible command structure add on if the driver supports atapi devices
     */
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
            // the command list only contains one header at the moment, so there is no need for an actual list
            // the command header points to the command list
            let mut first_cmd_header = Self::get_cmd_table_header(command_list_addr as *mut u8);

            // check if the port is available
            if Self::check_port_usable(port) != true {
                info!("ERR: port is not usable!");
                return;
            }

            // check if a command slot is ready
            let slot = self.find_cmd_slot(port);
            if slot == -1 {
                info!("ERR: no slot available!");
                return;
            }

            // calculate how many descriptors are needed for the access
            let mut descriptor_count: u32;
            let full_amt: u32 = byte_count / (BIGDESCRIPTOR);
            let rest = byte_count % (BIGDESCRIPTOR);
            if rest != 0 {
                descriptor_count = full_amt + 1;
            } else {
                descriptor_count = full_amt;
            }

            // calculate and allocate the necessary memory region to fit the whole command table with all descriptors
            let descriptors_per_page = 4096 / size_of::<HbaPhysicalRegionDescriptorTableEntry>();
            let allocated_space_for_descriptors =
                ((descriptor_count / descriptors_per_page as u32) + 1);

            let prdt_frames = frames::alloc(allocated_space_for_descriptors as usize);
            let prdt_start_addr = prdt_frames.start.start_address().as_u64();

            // create the command table in that allocated memory region
            let mut cmd_table = self.create_hba_cmd_table(
                byte_count,
                physical_dma,
                prdt_start_addr,
                descriptor_count,
            );
            (*cmd_table).commandFis = command_fis.clone();
            (*cmd_table).atapiCommand = atapi_command.clone();

            // calculate the length of the prdt
            let mut physical_region_descriptor_table_length;
            let full_amt = byte_count / (BIGDESCRIPTOR);
            let rest = byte_count % (BIGDESCRIPTOR);
            if rest != 0 {
                physical_region_descriptor_table_length = full_amt + 1;
            } else {
                physical_region_descriptor_table_length = full_amt;
            }

            let mut cmd_fis_len = size_of::<FisRegisterHostToDevice>() / size_of::<u32>();
            // the atapi field is only used for atapi commands that are not identify device
            let mut atapi = 0;
            if atapi_command[0] != 0 {
                atapi = 1;
            }
            //splitting the 64 bit base address into two 32 bit addresses
            let cmd_table_base_addr: u64 = prdt_start_addr; //cmd_table as u64;
            let upper_cmd_table_base_addr: u32 = (cmd_table_base_addr >> 32) as u32;
            let lower_cmd_table_base_addr = cmd_table_base_addr as u32;

            //the dword0 field contains atapi, cmd_fis_len and prdt_len
            //assemble the dword0
            let dword0 = (physical_region_descriptor_table_length << 16) as u32
                | (atapi << 5) as u32
                | cmd_fis_len as u32;
            (*first_cmd_header).dword0 = dword0;

            (*first_cmd_header).commandTableDescriptorBaseAddressUpper = upper_cmd_table_base_addr;
            (*first_cmd_header).commandTableDescriptorBaseAddress = lower_cmd_table_base_addr;
            //this filed works even with no value in qemu and on real hardware
            (*first_cmd_header).physicalRegionDescriptorByteCount = 0;

            //sending the command to the port
            let success = (*port).issueCommand(slot as u32);
            if !success {
                info!("ERR: issueCommand failed!")
            }

            //free the prdt because it is no longer needed
            frames::free(prdt_frames);
        }
    }

    /*
    performs a write access on a given port

    portnr is the number of the port on which the access needs to take place
    physical_dma is the address of the dma region where the data to write is
    byte_count is the amount of bytes that need to be processed
    command_fis is the command structure that needs to be processed
    atapi_command is a possible command structure add on if the driver supports atapi devices
    nearly the same as the read_from_device
     */
    pub unsafe fn write_to_device(
        &self,
        portnr: u32,
        mut physical_dma: u64,
        byte_count: u32,
        mut command_fis: [u8; 64],
        atapi_command: [u8; 16],
    ) -> bool {
        let mut port = (self.ports_start as *mut HbaPort).offset(portnr.try_into().unwrap());
        let mut command_list_addr = (*port).commandListBaseAddress as u64
            | (((*port).commandListBaseAddressUpper as u64) << 32);
        //get the command header
        let mut first_cmd_header = Self::get_cmd_table_header(command_list_addr as *mut u8);
        if Self::check_port_usable(port) != true {
            info!("ERR: port is not usable!");
            return false;
        }

        //get the slot
        let slot = self.find_cmd_slot(port);
        if slot == -1 {
            info!("ERR: no slot available!");
            return false;
        }

        //calculate the needed number of descriptors
        let mut descriptor_count: u32;
        let full_amt: u32 = byte_count / (BIGDESCRIPTOR);
        let rest = byte_count % (BIGDESCRIPTOR);
        if rest != 0 {
            descriptor_count = full_amt + 1;
        } else {
            descriptor_count = full_amt;
        }

        //allocate the descriptors
        let descriptors_per_page = 4096 / size_of::<HbaPhysicalRegionDescriptorTableEntry>();
        let allocated_space_for_descriptors =
            ((descriptor_count / descriptors_per_page as u32) + 1) as u64;

        //build the prdt
        let prdt_frames = frames::alloc(allocated_space_for_descriptors as usize);
        let prdt_start_addr = prdt_frames.start.start_address().as_u64();

        //connect the prdt with the command header
        let mut cmd_table =
            self.create_hba_cmd_table(byte_count, physical_dma, prdt_start_addr, descriptor_count);
        (*cmd_table).commandFis = command_fis.clone();
        (*cmd_table).atapiCommand = atapi_command.clone();

        //calculate the length of the prdt
        let mut physical_region_descriptor_table_length;
        let full_amt = byte_count / (BIGDESCRIPTOR);
        let rest = byte_count % (BIGDESCRIPTOR);
        if rest != 0 {
            physical_region_descriptor_table_length = full_amt + 1;
        } else {
            physical_region_descriptor_table_length = full_amt;
        }

        let mut cmd_fis_len = size_of::<FisRegisterHostToDevice>() / size_of::<u32>();
        let mut atapi = 0;
        if atapi_command[0] != 0 {
            atapi = 1;
        }
        //difference from read_from_device: the write bit has to be set
        //this change only matters for real hardware
        let write = 1;

        let cmd_table_base_addr: u64 = prdt_start_addr;
        let upper_cmd_table_base_addr: u32 = (cmd_table_base_addr >> 32) as u32;
        let lower_cmd_table_base_addr = cmd_table_base_addr as u32;

        //the dword0 field contains atapi, cmd_fis_len and prdt_len
        //assemble the dword0
        let dword0 = (physical_region_descriptor_table_length << 16) as u32
            | (atapi << 5) as u32
            | (write << 6) as u32
            | cmd_fis_len as u32;
        (*first_cmd_header).dword0 = dword0;

        (*first_cmd_header).commandTableDescriptorBaseAddressUpper = upper_cmd_table_base_addr;
        (*first_cmd_header).commandTableDescriptorBaseAddress = lower_cmd_table_base_addr;
        //this filed works even with no value in qemu and on real hardware
        (*first_cmd_header).physicalRegionDescriptorByteCount = 0;

        //sending the command to the port
        let success = (*port).issueCommand(slot as u32);
        if !success {
            info!("ERR: issueCommand hatte einen Fehler!")
        }

        //free the prdt because it is no longer needed
        frames::free(prdt_frames);
        return true;
    }

    /*
    the main access of this driver
    combines read and write requests

    portnr is the number of the port on which the access needs to take place
    max_capacity is the amount of sectors provided by the device
        this value can be found in the device info
    mode is the toggle between read and write
    buffer_addr is the address of the dma region where the results should be written into, or the data to write is
    start_sector is the number of the sector where the access should start
        the first sector of the device is 0
    sector_count is the amount of sectors needed for the access
     */
    pub unsafe fn performAtaIO(
        &self,
        portnr: u32,
        max_capacity: u64,
        mode: TransferMode,
        mut buffer_addr: u64,
        start_sector: u64,
        sector_count: u32,
    ) -> bool {
        //check that the position of the access is valid
        if start_sector + (sector_count as u64) > max_capacity {
            info!("ERR: AHCI trys to read/write out of bounds!");
            return false;
        }

        let mut command_fis = [0u8; 64];
        let mut atapi_cmd = [0u8; 16];

        //create the command fis
        let mut host_to_device_fis = FisRegisterHostToDevice {
            typ: 39,                     //typ = Host To Device
            port_mult_and_cmd_ctrl: 128, //only command control is 1
            command: 0,
            featureLow: 1,
            lba0: (start_sector & 0xff) as u8,
            lba1: (start_sector >> 8 & 0xff) as u8,
            lba2: (start_sector >> 16 & 0xff) as u8,
            device: 1 << 6, // LBA mode is on
            lba3: (start_sector >> 24 & 0xff) as u8,
            lba4: (start_sector >> 32 & 0xff) as u8,
            lba5: (start_sector >> 40 & 0xff) as u8,
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

            //calculate the buffer size
            let buffer_size = sector_count * SEKTORGROESSE;

            //pass everything into read function
            self.read_from_device(portnr, buffer_addr, buffer_size, command_fis, atapi_cmd);
        } else {
            host_to_device_fis.command = WRITE_DMA_EX;

            //copy the struct to the array
            unsafe {
                let ptr = command_fis.as_mut_ptr();
                let src = &host_to_device_fis as *const FisRegisterHostToDevice as *const u8;
                ptr::copy_nonoverlapping(src, ptr, size_of::<FisRegisterHostToDevice>());
            }

            //calculate the buffer size
            let buffer_size = sector_count * SEKTORGROESSE;

            //pass everything into write function
            let success =
                self.write_to_device(portnr, buffer_addr, buffer_size, command_fis, atapi_cmd);
            return success;
        }

        return true;
    }

    /*
    general version of the read command

    start_sector is the number of the first sector to access
    count is the amount of sectors to access
    buffer is the region where the result is written to
    portnr is the number of the port to access
    id_device is the device info of the device to access
     */
    unsafe fn read(
        &self,
        start_sector: u64,
        count: usize,
        buffer: &mut [u8],
        portnr: u32,
        id_device: &DeviceInfo,
    ) -> usize {
        //split the whole command payload into chunks the driver can work with
        let mut read_reps = count / SEKTORZAHL;
        let read_rest = count % SEKTORZAHL;
        if read_rest != 0 {
            read_reps = read_reps + 1;
        }

        //the end of the last chunk
        let mut current_offset = 0;

        //the real hardware has the value 0 for bytesPerSector
        //this driver only works with sector sizes of 512
        let mut sector_size = id_device.bytesPerSector;
        if sector_size == 0 {
            sector_size = 512;
        }

        for i in 0..read_reps {
            //calculate the size of the current chunk
            let remaining = count - (i * SEKTORZAHL);
            let mut sector_count = remaining;
            if sector_count > SEKTORZAHL {
                sector_count = SEKTORZAHL;
            }

            let read_bytes: u64 = (sector_size as usize * sector_count) as u64;

            //create the result region for the current chunk
            let region_buffer = AhciController::allocate_heap_region(read_bytes);
            let region_buffer_addr = region_buffer.start.start_address().as_u64();
            let max_capacity = id_device.lbaCapacity.try_into().unwrap();

            //send all data to the driver
            self.performAtaIO(
                portnr,
                max_capacity,
                TransferMode::READ,
                region_buffer_addr,
                start_sector + (i * SEKTORZAHL) as u64,
                sector_count as u32,
            );

            //copy the current result chunk into the right position of the whole result area
            let mut region_ptr = region_buffer_addr as *mut u8;
            let mybuffer = core::slice::from_raw_parts_mut(region_ptr, read_bytes as usize);
            let buffer_pos = buffer.as_mut_ptr().offset(current_offset);
            ptr::copy_nonoverlapping(region_ptr, buffer_pos, read_bytes as usize);

            //increase the chunk end for next iteration
            current_offset = current_offset + read_bytes as isize;

            //free the result area for the current iteration because it is no longer needed
            frames::free(region_buffer);
        }
        return count;
    }

    /*
    general version of the write command

    start_sector is the number of the first sector to access
    count is the amount of sectors to access
    buffer is the region where the data to write is
    portnr is the number of the port to access
    id_device is the device info of the device to access
     */
    unsafe fn write(
        &self,
        start_sector: u64,
        count: usize,
        buffer: &[u8],
        portnr: u32,
        id_device: &DeviceInfo,
    ) -> usize {
        //split the whole command payload into chunks the driver can work with
        let mut write_reps = count / SEKTORZAHL;
        let write_rest = count % SEKTORZAHL;
        if write_rest != 0 {
            write_reps = write_reps + 1;
        }

        //the real hardware has the value 0 for bytesPerSector
        //this driver only works with sector sizes of 512
        let mut sector_size = id_device.bytesPerSector;
        if sector_size == 0 {
            sector_size = 512;
        }

        for i in 0..write_reps {
            //calculate the size of the current chunk
            let remaining = count - (i * SEKTORZAHL);
            let mut sector_count = remaining;
            if sector_count > SEKTORZAHL {
                sector_count = SEKTORZAHL;
            }

            let write_bytes: u64 = (sector_size as usize * sector_count) as u64;

            //create the working region with the info to write
            let write_region = AhciController::allocate_heap_region(write_bytes);
            let write_region_addr = write_region.start.start_address().as_u64();

            let mut write_region_ptr = write_region_addr as *mut u8;
            let mut write_sl =
                core::slice::from_raw_parts_mut(write_region_ptr, write_bytes as usize);
            //copy the data to write from the given buffer to the working region
            for i in 0..write_sl.len() {
                write_sl[i] = buffer[i];
            }
            let max_capacity = id_device.lbaCapacity.try_into().unwrap();

            //send all data to the driver
            self.performAtaIO(
                portnr,
                max_capacity,
                TransferMode::WRITE,
                write_region_addr,
                start_sector + (i * SEKTORZAHL) as u64,
                sector_count as u32,
            );

            //free the working region
            frames::free(write_region);
        }
        count
    }

    /*
    add all devices to the block device trait
    this trait is important for further usage of this driver
     */
    pub unsafe fn init_all_ports_as_block_devices(&self) {
        let amt_port = self.check_cap_nr_of_ports();
        for i in 0..amt_port - 1 {
            info!("working on port {}", i);
            let current_port = (self.ports_start as *mut HbaPort).offset(i.try_into().unwrap());
            info!("init Port {} as block device", i);
            if Self::check_port_usable(current_port) {
                info!("port {} is usable", i);
                let ahci_drive = Arc::new(AHCIDrive::new(Arc::new(self.clone()), i));
                add_block_device("ata", ahci_drive);
            }
        }
    }

    /*****************************************************************************************************************************
    starting benchmarks

    the following code is especially designed for the sequential and random benchmark operations on qemu and real hardware
    the device access is the same in comparison to the read/write functions of the code above
    some mentionable code changes are the adding of a timer for benchmark purposes or the simplification of data to read or write

    *****************************************************************************************************************************/

    /*
    test if the device info from the identify_device function is correct by looking at the model, firmware and serial number
    if these values are correct, all the other values of the device info struct must be correct
    */
    unsafe fn test_identify_device_on_port(&self, portnr: u32) {
        let id_device = self.identify_device(portnr);
        let mut model = id_device.model.clone();

        //swap the bytes back because in the memory region the bytes are swapped
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
            "model is {}, firmware is {}, serial number is {}",
            model_str, firmware_str, serial_str
        );
    }

    /*
    the exact same code as the read function
    duplicate was necessary for testing purposes

    start_sector is the number of the first sector to access
    count is the amount of sectors to access
    buffer is the region where the result is written to
    portnr is the number of the port to access
    id_device is the device info of the device to access
     */
    unsafe fn test_read(
        &self,
        start_sector: u64,
        count: usize,
        buffer: &mut [u8],
        portnr: u32,
        id_device: &DeviceInfo,
    ) -> usize {

        //calculating the amount of chunks needed to deal with the whole payload
        let mut read_reps = count / SEKTORZAHL;
        let read_rest = count % SEKTORZAHL;

        if read_rest != 0 {
            read_reps = read_reps + 1;
        }
        let mut current_offset = 0;

        //calculate the size of a sector:
        //usually 512
        let mut sector_size = id_device.bytesPerSector;
        if sector_size == 0 {
            sector_size = 512;
        }

        for i in 0..read_reps {
            let remaining = count - (i * SEKTORZAHL);

            let mut sector_count = remaining;

            if sector_count > SEKTORZAHL {
                sector_count = SEKTORZAHL;
            }

            //create the memory space for the result of the driver
            let read_bytes: u64 = (sector_size as usize * sector_count) as u64;
            let region_buffer = AhciController::allocate_heap_region(read_bytes);
            let region_buffer_addr = region_buffer.start.start_address().as_u64();
            let max_capacity = id_device.lbaCapacity.try_into().unwrap();

            //send the data through the driver
            self.performAtaIO(
                portnr,
                max_capacity,
                TransferMode::READ,
                region_buffer_addr,
                start_sector + (i * SEKTORZAHL) as u64,
                sector_count as u32,
            );

            //copy the result of the driver to the right position of the whole result
            let mut region_ptr = region_buffer_addr as *mut u8;
            let mybuffer = core::slice::from_raw_parts_mut(region_ptr, read_bytes as usize);
            let buffer_pos = buffer.as_mut_ptr().offset(current_offset);
            ptr::copy_nonoverlapping(region_ptr, buffer_pos, read_bytes as usize);

            current_offset = current_offset + read_bytes as isize;

            //free the memory of the current chunk
            frames::free(region_buffer);
        }
        return count;
    }

    /*
    nearly the same code as in the write function
    because in benchmarks only one number will be written into the device, the variable buffer was exchanged to a single number

    portnr is the number of the port to access
    start_sector is the number of the first sector to access
    count is the amount of sectors to access
    nr_to_write is the number that should be written into the sectors
        ->difference to write function    
    id_device is the device info of the device to access
     */
    unsafe fn test_write(
        &self,
        portnr: u32,
        start_sector: u64,
        count: usize,
        nr_to_write: u8,
        id_device: &DeviceInfo,
    ) -> isize {

        //ensure that in qemu only the port 1 is the one to use
        //no fatal error because on hardware port 0 is the right one to use
        if portnr == 0 {
              info!("Warning: writing into QEMU boot image. Ignore if on real hardware!");
        }

        //calculating the amount of chunks needed to deal with the whole payload
        let mut write_reps = count / SEKTORZAHL;
        let write_rest = count % SEKTORZAHL;

        if write_rest != 0 {
            write_reps = write_reps + 1;
        }


        let mut write_time: isize = 0;
        for i in 0..write_reps {
            let mut sector_size = id_device.bytesPerSector;
            if sector_size == 0 {
                sector_size = 512;
            }
            let remaining = count - (i * SEKTORZAHL);

            let mut sector_count = remaining;

            if sector_count > SEKTORZAHL {
                sector_count = SEKTORZAHL;
            }

            //create the buffer with data that the driver needs in order to write the data
            let write_bytes: u64 = (sector_size as usize * sector_count) as u64;
            let write_region = AhciController::allocate_heap_region(write_bytes);
            let write_region_addr = write_region.start.start_address().as_u64();

            let mut write_region_ptr = write_region_addr as *mut u8;
            let mut write_sl =
                core::slice::from_raw_parts_mut(write_region_ptr, write_bytes as usize);
            
            //write the experimental data into the buffer
            for i in 0..write_sl.len() {
                write_sl[i] = nr_to_write;
            }
            let max_capacity = id_device.lbaCapacity.try_into().unwrap();

            //start timer
            let start_time = sys_get_system_time();

            //send the data to the driver
            let help = self.performAtaIO(
                portnr,
                max_capacity,
                TransferMode::WRITE,
                write_region_addr,
                start_sector + (i * SEKTORZAHL) as u64,
                sector_count as u32,
            );

            //end timer
            let end_time = sys_get_system_time();

            //calculate the time to write the data
            write_time = write_time + (end_time - start_time);
            frames::free(write_region);
        }

        write_time
    }

    
    /*****************************************************************************************************************************
    benchmark scenarios

    the following code combines the upper functions to these benchmark scenarios:

    sequential read
    sequential write
    random read
    random write

    each scenario comes with a single function and a benchmark function
    the single function is used for each benchmark function
    the benchmark functions are for easy access in the init function
    sequential scenarios also verify that the read and write operations give the right results
    
    *****************************************************************************************************************************/

    /*
    test if a read amount of sectors has the right data, starting from sector 0

    sector_count is the number of sectors to read
    correct_arr is the expected result
    id_device is the device info needed for the driver
    port_nr is the port number where the read should take place

    if the read sectors does not fit with the correct array, it returns -1
    if the read was successfull, it returns the needed time
     */
    
    pub unsafe fn benchmark_check_single_read(
        &self,
        sector_count: u32,
        correct_arr: &[u8],
        id_device: &DeviceInfo,
        port_nr: u32,
    ) -> isize {

        let sector_size = SEKTORGROESSE;

        //create the buffer for the result
        let read_bytes: u64 = (sector_size * sector_count) as u64;
        let single_region = AhciController::allocate_heap_region(read_bytes);
        let single_region_ptr = single_region.start.start_address().as_u64() as *mut u8;
        let buffer = core::slice::from_raw_parts_mut(single_region_ptr, read_bytes as usize);

        //start timer
        let start_time = sys_get_system_time();

        //perform the read using the driver
        let read_bytes = self.test_read(0, sector_count as usize, buffer, port_nr, id_device);

        //end timer
        let end_time = sys_get_system_time();

        //calculate the needed time
        let mut read_time = end_time - start_time;

        //check if the result matches with the expected result
        let mut equal = true;
        for i in 0..buffer.len() {
            if buffer[i] != correct_arr[i] {
                info!(
                    "wrong position {} has the value {} while expected was {}",
                    i, buffer[i], correct_arr[i]
                );
                equal = false;
                break;
            }
        }

        //free the memory region for the result since it is no longer needed 
        frames::free(single_region);

        if equal {
            read_time
        } else {
            -1 as isize
        }
    }

    /*
    one read at a random given position
    because this type of benchmark only gets testet after the sequential one is done, we can assume that the read sectors are correct

    start_sector is the random sector number selected for the read access
    id_device is the device info needed for the driver
    port_nr is the number of the port where the read should take place

     */

    pub unsafe fn benchmark_random_single_read(
        &self,
        start_sector: u64,
        id_device: &DeviceInfo,
        port_nr: u32,
    ) -> isize {

        let read_bytes: u64 = SEKTORGROESSE as u64;

        //create the memory region for the result
        let single_region = AhciController::allocate_heap_region(read_bytes);
        let single_region_ptr = single_region.start.start_address().as_u64() as *mut u8;
        let buffer = core::slice::from_raw_parts_mut(single_region_ptr, read_bytes as usize);

        //start timer
        let start_time = sys_get_system_time();

        //perform the read using the driver
        let read_bytes = self.test_read(start_sector, 1, buffer, port_nr, id_device);

        //end timer
        let end_time = sys_get_system_time();

        //free the memory region for the result since it is no longer needed 
        frames::free(single_region);

        //calculate the needed time
        (end_time - start_time) as isize
    }

    /*
    the complete benchmark in the sequential scenario

    sector_count is the number of sectors to read
    repetitions should be 100, else the time splitting at the end need ajustment
    port_nr is the port number the access takes place
     */

    pub unsafe fn benchmark_read(&self, sector_count: u32, repetitions: u32, port_nr: u32) {
         info!(
            "start read benchmark, with {} sectors in a sequence and {} repetitions",
            sector_count, repetitions
        );

        let mut read_times: Vec<isize> = Vec::new();

        //get the device info
        let id_device = self.identify_device(port_nr);

        //create the memory region for the expected result
        let sector_size = SEKTORGROESSE; 
        let read_bytes: u64 = (sector_size * sector_count) as u64;
        let single_region = AhciController::allocate_heap_region(read_bytes);
        info!("allocate buffer with {} bytes in size", read_bytes);
        let single_region_ptr = single_region.start.start_address().as_u64() as *mut u8;
        let buffer = core::slice::from_raw_parts_mut(single_region_ptr, read_bytes as usize);

        //get the expected result
        let correct_read_bytes =
            self.test_read(0, sector_count as usize, buffer, port_nr, &id_device);

        let mut full_time_ms = 0;
        let mut amt_success = 0;

        //test the sequential reading against the expected result
        for i in 0..repetitions {
            let single_result =
                self.benchmark_check_single_read(sector_count, &buffer, &id_device, port_nr);
            if single_result != -1 {
                full_time_ms += single_result;
                read_times.push(single_result);
                amt_success += 1;
            }
        }

        info!(
            "finished read benchmark, with {} sectors in a sequence and {} repetitions",
            sector_count, repetitions
        );
        info!(
            "managed to read {} of {} times successfully with a complete time of {} ms",
            amt_success, repetitions, full_time_ms
        );

        //split the 100 repetitions so minicom shows every single value 
        let q1 = &read_times[0..10];
        let q2 = &read_times[10..20];
        let q3 = &read_times[20..30];
        let q4 = &read_times[30..40];
        let q5 = &read_times[40..50];
        let q6 = &read_times[50..60];
        let q7 = &read_times[60..70];
        let q8 = &read_times[70..80];
        let q9 = &read_times[80..90];
        let q10 = &read_times[90..100];

        //print the results in 10 rows so there is enough space for every result
        info!(
            "the reading times are: \n{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}",
            q1, q2, q3, q4, q5, q6, q7, q8, q9, q10
        );

        //free the result memory region since it is no longer needed
        frames::free(single_region);
    }

    /*
    the complete random read benchmark

    repetitions is the number of repetitions wanted
    port_nr is the port number the access takes place
     */

    pub unsafe fn benchmark_random_read(&self, repetitions: u32, port_nr: u32) -> isize {
        info!(
            "start random read benchmark, with one sector at a random position and {} repetitions",
            repetitions
        );
        let id_device = self.identify_device(port_nr);
        let mut full_time_ms = 0;

        //generate the random number generator using a fixed seed
        let mut small_rng = SmallRng::seed_from_u64(5);

        for i in 0..repetitions {
            // creating the random position
            // the random position is within all of the sectors
            let rand_pos = small_rng.next_u64();
            let max_amt_of_sectors = (id_device.lbaCapacity - 1) as u64;
            let fitting_pos = max_amt_of_sectors & rand_pos;

            //read one sector at the random position
            //the single random read function already does the timing
            let single_result = self.benchmark_random_single_read(fitting_pos, &id_device, port_nr);
            full_time_ms += single_result;
        }
        info!(
            "finished random read benchmark, with one sector at a random position and {} repetitions",
            repetitions
        );
        info!(
            "managed to read with a complete time of {} ms",
            full_time_ms
        );
        full_time_ms
    }

    
    /*
    test if a given number of sectors are written successfully to the device, starting from sector 0

    sector_count is the number of sectors to write
    id_device is the device info needed for the driver
    port_nr is the port number where the write should take place

    if the written sectors dont have the right value it returns -1
    if the write was successfull, it returns the needed time
     */

    pub unsafe fn benchmark_check_single_write(
        &self,
        sector_count: usize,
        id_device: &DeviceInfo,
        port_nr: u32,
    ) -> isize {
        //write the number into the sectors
        let work_time = self.test_write(port_nr, 0, sector_count, 5, &id_device);

        //create the buffer for the read control
        let sector_size = SEKTORGROESSE;
        let read_bytes: u64 = (SEKTORGROESSE * sector_count as u32) as u64;
        let single_region = AhciController::allocate_heap_region(read_bytes);
        let single_region_ptr = single_region.start.start_address().as_u64() as *mut u8;
        let buffer = core::slice::from_raw_parts_mut(single_region_ptr, read_bytes as usize);

        //read from the written sectors
        //the result should only contain the nr_to_write value
        let read = self.test_read(0, sector_count as usize, buffer, port_nr, &id_device);
        let mut count = 0;
        let mut count_bad = 0;
        let mut success = true;
        for i in 0..buffer.len() {
            if buffer[i] != 5 {
                info!(
                    "ERR: test_read has a different value on position: {}. The value should be 5 but is: {} !",
                    i, buffer[i]
                );
                success = false;
                count_bad += 1;
                break;
            } else {
                count += 1;
            }
        }

        // reset the sectors to another value, so in a repetition another write command has the same payload
        self.test_write(port_nr, 0, sector_count, 8, &id_device);

        //free the read control buffer since it is no longer needed
        frames::free(single_region);

        //return the result of the benchmark
        if success { work_time } else { -1 }
    }

    /*
    one write at a random given position
    because this type of benchmark only gets testet after the sequential one is done, we can assume that the written sectors are correct

    start_sector is the random sector number selected for the write access
    id_device is the device info needed for the driver
    port_nr is the number of the port where the write should take place
        
     */

    pub unsafe fn benchmark_random_single_write(
        &self,
        start_sector: u64,
        id_device: &DeviceInfo,
        port_nr: u32,
    ) -> isize {
        let work_time = self.test_write(port_nr, start_sector, 1, 5, id_device);

        // reset the sectors to another value
        self.test_write(port_nr, start_sector, 1, 8, id_device);
        work_time
    }

    /*
    the complete benchmark in the sequential scenario

    sector_count is the number of sectors to write
    repetitions should be 100, else the time splitting at the end need ajustment
    port_nr is the port number the access takes place
     */

    pub unsafe fn benchmark_write(&self, sector_count: usize, repetitions: u32, port_nr: u32) {
        info!(
            "start write benchmark, with {} sectors in a sequence and {} repetitions",
            sector_count, repetitions
        );

        let mut write_times: Vec<isize> = Vec::new();

        //get the device info for the driver
        let id_device = self.identify_device(port_nr);
        let mut full_time_ms = 0;
        let mut amt_success = 0;

        //running the benchmark
        for i in 0..repetitions {
            let single_result =
                self.benchmark_check_single_write(sector_count, &id_device, port_nr);
            if single_result != -1 {
                full_time_ms += single_result;
                amt_success += 1;
                write_times.push(single_result);
            }
        }
        info!(
            "finished write benchmark, with {} sectors in a sequence and {} repetitions",
            sector_count, repetitions
        );
        info!(
            "managed to write {} of {} times successfully with a complete time of {} ms",
            amt_success, repetitions, full_time_ms
        );

        //split the 100 repetitions so minicom shows every single value 
        let q1 = &write_times[0..10];
        let q2 = &write_times[10..20];
        let q3 = &write_times[20..30];
        let q4 = &write_times[30..40];
        let q5 = &write_times[40..50];
        let q6 = &write_times[50..60];
        let q7 = &write_times[60..70];
        let q8 = &write_times[70..80];
        let q9 = &write_times[80..90];
        let q10 = &write_times[90..100];

        //print the results in 10 rows so there is enough space for every result
        info!(
            "the write times are: \n{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}#\n#{:?}",
            q1, q2, q3, q4, q5, q6, q7, q8, q9, q10
        );
    }


    /*
    the complete random write benchmark

    repetitions is the number of repetitions wanted
    port_nr is the port number the access takes place
     */

    pub unsafe fn benchmark_random_write(&self, repetitions: u32, port_nr: u32) -> isize {
        let id_device = self.identify_device(port_nr);
        let mut full_time_ms = 0;

        //generate the random nr_generator using a fixed seed
        let mut small_rng = SmallRng::seed_from_u64(5);

        for i in 0..repetitions {
            // creating the random position
            // the random position is within all of the sectors
            let rand_pos = small_rng.next_u64();
            let max_amt_of_sectors = (id_device.lbaCapacity - 1) as u64;
            let fitting_pos = max_amt_of_sectors & rand_pos;
            //write one sector at the random position
            let single_result =
                self.benchmark_random_single_write(fitting_pos, &id_device, port_nr);
            full_time_ms += single_result;
        }
        info!(
            "finished random write benchmark, with one sector at a random position and {} repetitions",
            repetitions
        );
        info!(
            "managed to write with a complete time of {} ms",
            full_time_ms
        );
        full_time_ms
    }
}


/*****************************************************************************************************************************
    issue command

    this code belongs to the driver but need to be executed on the port
    
    each port can have multiple command slots but at the moment only slot 0 is needed
    
*****************************************************************************************************************************/


#[allow(warnings)]
impl HbaPort {
    pub fn issueCommand(&mut self, slot: u32) -> bool {
        //define all values for the port
        const COMMAND_TIMEOUT: isize = 10000;
        const BIG_TIMEOUT: isize = 50000;
        const BUSY: u32 = 128;
        const DATA_TRANSFER_REQUESTED: u32 = 8;
        const TASK_FILE_ERROR: u32 = 1 << 30;
        let mut timeout = sys_get_system_time() + COMMAND_TIMEOUT;

        // wait while device is busy
        while (self.taskFileData & (BUSY | DATA_TRANSFER_REQUESTED)) > 0 {
            if (sys_get_system_time() >= timeout) {
                info!("ERR: issue command reached a timeout before trying to complete the given command!");
                return false;
            }
            //continue with other threads while the device is working
            scheduler().switch_thread_no_interrupt();
        }

        // issue command by setting the right bit
        self.commandIssue = 1 << slot;


        // wait for command completion
        timeout = sys_get_system_time() + BIG_TIMEOUT;
        while true {
            let test = self.sataError;

            //the command is completed
            if ((self.commandIssue & (1 << slot)) == 0) {
                break;
            }

            //there was an error during the command completion
            if (self.interruptStatus & TASK_FILE_ERROR) > 0 {
                info!("ERR: issue command could not complete the command!");
                return false;
            }

            //the command completion takes too long so the command is cancelled
            if (sys_get_system_time() >= timeout) {
                info!("ERR: issue command reached a timeout while trying to complete the given command!");
                return false;
            }

            //continue with other threads while the device is working
            scheduler().switch_thread_no_interrupt();
        }
        true
    }
}

/*
definition of an ahci drive which is needed for the block device trait
*/

pub struct AHCIDrive {
    controller: Arc<AhciController>,
    info: DeviceInfo,
    portnr: u32,
}

/*
constructor of the ahci drive for the device trait
*/

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

/*
implementation for the block device trait
*/

impl BlockDevice for AHCIDrive {
    fn read(&self, sector: u64, count: usize, buffer: &mut [u8]) -> usize {
        unsafe {
            self.controller
                .read(sector, count, buffer, self.portnr, &self.info);
            count
        }
    }

    fn write(&self, sector: u64, count: usize, buffer: &[u8]) -> usize {
        unsafe {
            self.controller
                .write(sector, count, buffer, self.portnr, &self.info);
            count
        }
    }

    fn sector_count(&self) -> u64 {
        self.info.lbaCapacity as u64
    }

    fn sector_size(&self) -> u16 {
        SEKTORGROESSE as u16
    }
}


