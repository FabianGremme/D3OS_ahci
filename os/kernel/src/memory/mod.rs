pub mod vmm;
pub mod vma;
pub mod pages;
pub mod frames;

pub mod nvmem;

pub mod heap;
pub mod stack;
pub mod acpi_handler;

pub mod ahciController;

#[derive(PartialEq)]
#[derive(Clone, Copy)]
pub enum MemorySpace {
    Kernel,
    User
}

pub const PAGE_SIZE: usize = 0x1000;