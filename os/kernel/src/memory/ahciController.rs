use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::any::Any;
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
use crate::syscall::sys_time::{sys_get_system_time, wait_ms};

const MASS_STORAGE_DEVICE: BaseClass = 0x01;
const SATA_CONTROLLER: SubClass = 0x06;

enum BiosHandoffFlags {
    BIOS_OWNED_SEMAPHORE = 1 << 0,
    OS_OWNED_SEMAPHORE = 1 << 1,
    SMI_ON_OWNERSHIP_CHANGE = 1 << 2,
    OS_OWNERSHIP_CHANGE = 1 << 3,
    BIOS_BUSY = 1 << 4
}

//wird verwendet, um die command engine zu starten und zu stoppen

const START: u32 = 1 << 0;
const FIS_RECIVE_ENABLE: u32 = 1 << 4;
const FIS_RECEIVE_RUNNING: u32 = 1 << 14;
const COMMAND_LIST_RUNNING: u32 = 1 << 15;


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
    first: u32,//ReadWrite<u32, D0::Register>,

    // DWORD 1
    physicalRegionDescriptorByteCount: u32,

    // DWORD 2-3
    commandTableDescriptorBaseAddress: u32,
    commandTableDescriptorBaseAddressUpper: u32,

    // DWORD 4-7
    reserved: [u32;4],
}
/*register_bitfields![u32,D0[
    commandFisLength OFFSET(0) NUMBITS(5) [],
    atapi OFFSET(5) NUMBITS(1) [],
    write OFFSET(6) NUMBITS(1) [],
    prefetchable OFFSET(7) NUMBITS(1) [],
    reset OFFSET(8) NUMBITS(1) [],
    bist OFFSET(9) NUMBITS(1) [],
    clearBusyOnOk OFFSET(10) NUMBITS(1) [],
    reserved1 OFFSET(11) NUMBITS(1) [],
    portMultiplierPort OFFSET(12) NUMBITS(4) [],
    physicalRegionDescriptorTableLength OFFSET(16) NUMBITS(16) [],
    ]];*/


//Laut Bachelorarbeit soll eine combined HBA Command Table aus einer cmd_table und 8 Einheiten der Liste entstehen
struct combined_HBA_CommandTable{
    cmd_table: HbaCommandTable,
    physicalRegionDescriptorTable: Vec<HbaPhysicalRegionDescriptorTableEntry>,
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct HbaPhysicalRegionDescriptorTableEntry {
    dataBaseAddress: u32,
    dataBaseAddressUpper: u32,
    reserved1: u32,
    rest: u32,

    //uint32_t dataByteCount: 22;
    //uint32_t reserved2: 9;
    //uint32_t interruptOnCompletion: 1;
}
#[allow(warnings)]
#[repr(C, packed)]
#[derive(Debug)]
struct HbaCommandTable {
    commandFis: [u8;64],
    atapiCommand: [u8;16],
    reserved: [u8;48],
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

    pub fn check_nr_of_command_slots(&self){
        let cap = self.hba_regs.hostCapabilities;
        let nr_of_cmds = Self::general_bitlen_reader(cap, 8, 5);
        info!("laut capabilities werden {} Command slots unterstützt.", nr_of_cmds);
    }

    pub fn map_command_components(&self){
        //der Port muss noch zurückgesetzt werden, aber das kommt noch
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
        //der Port muss noch zurückgesetzt werden, aber das kommt noch
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
}

// Todo:
//Comand Liste anschauen (es werden 31 command slots unterstützt) (es wird kein weiterer gefunden)


//tock registers (anschauen) (passt nicht)

//prdt mappen und genauer anschauen:
//  das Feld prdt, welches aktuell noch zusammen ist, muss auf 8 begrenzt werden (fertig)

//command table mit Werten befülen / mapping testen (command table ist zu groß und unbestimmt, als dass sie mit Werten gefüllt werden kann. aktuell ist die prdtl = 1)

//command table mit allen 32 headern versuchen zu allocaten

// Warum wird im HHU OS ein Fehler mit F zugeschrieben, als reset?

//device erkennung impl









