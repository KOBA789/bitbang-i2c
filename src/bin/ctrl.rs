#![no_std]
#![no_main]

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    I2C,
};
use defmt_rtt as _;
use embedded_hal::i2c::I2c;
use panic_probe as _;
use rp_pico as bsp;
use fugit::RateExtU32 as _;

#[bsp::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
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

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio18.reconfigure(),
        pins.gpio19.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut buf = [0u8; 4];
    loop {
        //i2c.read(0x01u8, &mut buf).ok();
        //defmt::println!("{=[u8]:02x}", buf);
        i2c.write_read(0x6Bu8, &[0xDE, 0xAD, 0xBE, 0xEF], &mut buf).ok();
        //i2c.write(0x01u8, &[0xDE, 0xAD, 0xBE, 0xEF]).ok();
        cortex_m::asm::delay(125_000_000);
    }
}
