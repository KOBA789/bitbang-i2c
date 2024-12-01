#![no_std]

use core::ptr::{addr_of, addr_of_mut};

use hal::gpio::{
    bank0::{Gpio0, Gpio1},
    FunctionSioInput, Pin, PullNone,
};
use rp_pico::hal::{
    self,
    multicore::{Core, Stack},
};

const GPIO_OUT_SET: *mut u32 = 0xD000_0014 as *mut u32;
const GPIO_OUT_CLR: *mut u32 = 0xD000_0018 as *mut u32;
const GPIO_OE_SET: *mut u32 = 0xD000_0024 as *mut u32;
const GPIO_OE_CLR: *mut u32 = 0xD000_0028 as *mut u32;

const SDA_PIN: u8 = 0;
const SCL_PIN: u8 = 1;

// for debugging
#[inline(always)]
#[allow(unused)]
fn toggle() {
    unsafe { core::ptr::write_volatile(0xD000_001C as *mut u32, 0x0001_0000) }
}

#[inline(always)]
fn scl_sda() -> u8 {
    unsafe { core::ptr::read_volatile(0xD000_0004 as *const u32) as u8 & 0b11 }
}

#[inline(always)]
fn gpio_out_set(pin: u8) {
    unsafe { core::ptr::write_volatile(GPIO_OUT_SET, 1 << pin) }
}

#[inline(always)]
fn gpio_out_clr(pin: u8) {
    unsafe { core::ptr::write_volatile(GPIO_OUT_CLR, 1 << pin) }
}

#[inline(always)]
fn gpio_oe_set(pin: u8) {
    unsafe { core::ptr::write_volatile(GPIO_OE_SET, 1 << pin) }
}

#[inline(always)]
fn gpio_oe_clr(pin: u8) {
    unsafe { core::ptr::write_volatile(GPIO_OE_CLR, 1 << pin) }
}

#[inline(always)]
fn set_sda_as_output() {
    gpio_oe_set(SDA_PIN);
}

#[inline(always)]
fn set_sda_as_input() {
    gpio_oe_clr(SDA_PIN);
}

#[inline(always)]
fn set_sda_high() {
    gpio_out_set(SDA_PIN);
}

#[inline(always)]
fn set_sda_low() {
    gpio_out_clr(SDA_PIN);
}

#[inline(always)]
fn set_scl_as_output() {
    gpio_oe_set(SCL_PIN);
}

#[inline(always)]
fn set_scl_as_input() {
    gpio_oe_clr(SCL_PIN);
}

#[inline(always)]
fn set_scl_low() {
    gpio_out_clr(SCL_PIN);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Byte {
    Data(u8),
    Start,
    Stop,
}

#[inline(always)]
fn read_bit() -> Byte {
    let bit = wait_scl_high();
    loop {
        let pins = scl_sda();
        if pins & 0b10 == 0 {
            return Byte::Data(bit);
        }
        if pins & 0b01 != bit {
            if bit == 0 {
                return Byte::Stop;
            } else {
                wait_scl_low();
                return Byte::Start;
            }
        }
    }
}

#[inline(always)]
fn read_byte() -> Byte {
    let mut byte = 0u8;
    for _ in 0..8 {
        match read_bit() {
            Byte::Data(bit) => {
                byte <<= 1;
                byte |= bit;
            }
            cond => return cond,
        }
    }
    reply_ack();
    Byte::Data(byte)
}

#[inline(always)]
fn reply_ack() {
    delay(70);
    set_sda_low();
    set_sda_as_output();
    wait_scl_high();
    wait_scl_low();
    set_sda_as_input();
}

// cortex_m::asm::delay を使うと FLASH 内のコードを
// 呼び出してしまって遅延が不定になってしまうのでこれを使う
#[inline(always)]
fn delay(cycles: u32) {
    let real_cycles = 1 + cycles / 2;
    unsafe {
        core::arch::asm!(
            "1:",
            "subs {}, #1",
            "bne 1b",
            inout(reg) real_cycles => _,
            options(nomem, nostack),
        )
    };
}

#[inline(always)]
fn wait_scl_high() -> u8 {
    loop {
        let pins = scl_sda();
        if pins & 0b10 == 0 {
            continue;
        }
        break pins & 0b01;
    }
}

#[inline(always)]
fn wait_scl_low() {
    while scl_sda() & 0b10 != 0 {}
}

#[inline(always)]
fn write_byte(mut byte: u8) -> bool {
    set_sda_as_output();
    for _ in 0..8 {
        let bit = byte & (1 << 7);
        byte <<= 1;
        if bit == 0 {
            set_sda_low();
        } else {
            set_sda_high();
        }
        set_scl_as_input();
        wait_scl_high();
        wait_scl_low();
    }
    set_sda_as_input();

    wait_scl_high();
    let ack = scl_sda() & 0b01 == 0;
    wait_scl_low();
    ack
}

#[inline(always)]
fn clock_stretch() {
    set_scl_low();
    set_scl_as_output();
}

#[inline(always)]
fn fifo_write(value: u32) {
    unsafe { core::ptr::write_volatile(0xD000_0054 as *mut u32, value) }
}

#[inline(always)]
fn fifo_write_blocking(non_block_value: u32, block_value: u32) {
    if fifo_status() & 0b10 == 0 {
        // full
        while fifo_status() & 0b10 == 0 {
            fifo_write(block_value);
        }
    } else {
        fifo_write(non_block_value);
    }
}

#[inline(always)]
fn fifo_read() -> u32 {
    unsafe { core::ptr::read_volatile(0xD000_0058 as *const u32) }
}

#[inline(always)]
fn fifo_status() -> u32 {
    unsafe { core::ptr::read_volatile(0xD000_0050 as *const u32) }
}

#[inline(always)]
fn addr(byte: u8) -> (u8, bool) {
    let rw = byte & 1 == 1;
    let addr = byte >> 1;
    (addr, rw)
}

enum State {
    Init,
    Addr,
    Write,
}

#[link_section = ".bbi2c_run"]
#[no_mangle]
fn bbi2c_run(
    _sda: Pin<Gpio0, FunctionSioInput, PullNone>,
    _scl: Pin<Gpio1, FunctionSioInput, PullNone>,
) -> ! {
    let mut state = State::Init;
    loop {
        loop {
            match read_byte() {
                Byte::Data(byte) => match state {
                    State::Write => {
                        fifo_write_blocking(byte as u32, 0xFF00);
                    }
                    State::Addr => {
                        let (_addr, is_read) = addr(byte);
                        fifo_write_blocking(byte as u32 | 1 << 8, 0xFF00);
                        if is_read {
                            break;
                        } else {
                            state = State::Write;
                        }
                    }
                    State::Init => continue,
                },
                Byte::Start => {
                    state = State::Addr;
                    fifo_write_blocking(2 << 8, 0xFF00);
                    continue;
                }
                Byte::Stop => {
                    fifo_write_blocking(3 << 8, 0xFF00);
                    continue;
                }
            }
        }
        loop {
            if fifo_status() & 0b1 == 0 {
                clock_stretch();
                fifo_write_blocking(4 << 8, 4 << 8);
            }
            while fifo_status() & 0b1 == 0 {}
            let byte = fifo_read() as u8;
            if !write_byte(byte) {
                break;
            }
        }
        // Flush the FIFO
        while fifo_status() & 0b1 != 0 {
            fifo_read();
        }
    }
}

#[link_section = ".bbi2c_stack"]
#[no_mangle]
static mut BBI2C_STACK: Stack<1024> = Stack::new();

pub fn spawn(
    core: &mut Core<'_>,
    sda: Pin<Gpio0, FunctionSioInput, PullNone>,
    scl: Pin<Gpio1, FunctionSioInput, PullNone>,
) {
    unsafe {
        extern "C" {
            static mut __sbbi2c: u8;
            static mut __ebbi2c: u8;
            static __sibbi2c: u8;
        }
        let sbbi2c = addr_of_mut!(__sbbi2c);
        let ebbi2c = addr_of_mut!(__ebbi2c);
        let sibbi2c = addr_of!(__sibbi2c);
        let count = ebbi2c as usize - sbbi2c as usize;
        core::ptr::copy_nonoverlapping(sibbi2c, sbbi2c, count);
    }

    let stack = unsafe { &mut BBI2C_STACK.mem };
    core.spawn(stack, move || bbi2c_run(sda, scl)).unwrap();
}
