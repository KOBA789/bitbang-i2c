#![no_std]
#![no_main]

use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use defmt::Format;
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

#[bsp::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let _clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _sda = pins.gpio0.into_floating_input();
    let _scl = pins.gpio1.into_floating_input();

    let _debug = pins.gpio16.into_push_pull_output();

    main_loop();
}

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
fn set_sda_as_output() {
    unsafe { core::ptr::write_volatile(0xD000_0024 as *mut u32, 1) }
}

#[inline(always)]
fn set_sda_as_input() {
    unsafe { core::ptr::write_volatile(0xD000_0028 as *mut u32, 1) }
}

#[inline(always)]
fn set_sda_high() {
    unsafe { core::ptr::write_volatile(0xD000_0014 as *mut u32, 1) }
}

#[inline(always)]
fn set_sda_low() {
    unsafe { core::ptr::write_volatile(0xD000_0018 as *mut u32, 1) }
}

#[inline(always)]
fn set_scl_as_output() {
    unsafe { core::ptr::write_volatile(0xD000_0024 as *mut u32, 1 << 1) }
}

#[inline(always)]
fn set_scl_as_input() {
    unsafe { core::ptr::write_volatile(0xD000_0028 as *mut u32, 1 << 1) }
}

#[inline(always)]
fn set_scl_low() {
    unsafe { core::ptr::write_volatile(0xD000_0018 as *mut u32, 1 << 1) }
}

#[derive(Format, Clone, Copy, PartialEq, Eq)]
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
    cortex_m::asm::delay(70);
    set_sda_low();
    set_sda_as_output();
    wait_scl_high();
    wait_scl_low();
    set_sda_as_input();
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

#[link_section = ".data"]
#[no_mangle]
fn main_loop() -> ! {
    let mut state = State::Init;
    let mut buf = heapless::Vec::<u8, 64>::new();
    let mut last_addr = 0x00;
    loop {
        loop { // receive loop
            match read_byte() {
                Byte::Data(byte) => match state {
                    State::Write => {
                        // TODO: Push byte to FIFO
                        buf.push(byte).ok();
                    },
                    State::Addr => {
                        let (addr, is_read) = addr(byte);
                        // TODO: Push addr to FIFO
                        last_addr = addr;
                        if is_read {
                            // READ
                            break;
                        } else {
                            // WRITE
                            state = State::Write;
                        }
                    },
                    State::Init => continue,
                },
                Byte::Start => {
                    state = State::Addr;
                    continue;
                },
                Byte::Stop => {
                    defmt::println!("addr: {:02x}, buf: {=[u8]:02x}", last_addr, buf);
                    buf.clear();
                    continue;
                },
            }
        }
        clock_stretch();
        cortex_m::asm::delay(125_000); // 1ms
        while write_byte(0xDE) {}
    }
}
