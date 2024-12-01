#![no_std]
#![no_main]

use bsp::hal::{
    clocks::init_clocks_and_plls, multicore::Multicore, pac, sio::Sio, watchdog::Watchdog,
};
use defmt_rtt as _;
use panic_probe as _;
use rp_pico as bsp;

#[bsp::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda = pins.gpio0.into_floating_input();
    let scl = pins.gpio1.into_floating_input();

    let _debug = pins.gpio16.into_push_pull_output();

    bitbang_i2c::spawn(core1, sda, scl);

    loop {
        if let Some(word) = sio.fifo.read() {
            defmt::println!("{:04x}", word);
            if word == 0x0400 {
                sio.fifo.write(0xBE);
            }
        }
        //sio.fifo.write(0xBE);
        cortex_m::asm::delay(1250);
    }
}
