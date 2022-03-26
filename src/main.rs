#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_halt as _;

// Time handling traits
use embedded_time::rate::*;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

// Pull in any important traits
use bsp::hal::prelude::*;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use bsp::hal;
use bsp::hal::pac;

// Some traits we need
use bsp::hal::clocks::Clock;
use core::fmt::Write;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// #[link_section = ".boot2"]
// #[used]
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};
// USB Communications Class Device support
use usbd_serial::SerialPort;
// Convert a number to a string
use numtoa::NumToA;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set the pins to their default state
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Init LED
    let mut led_pin = pins.led.into_push_pull_output();

    // Init Switchs
    let sw_a = pins.gpio15.into_pull_up_input();
    let mut sw_a_flag = false;
    let sw_b = pins.gpio17.into_pull_up_input();
    let mut sw_b_flag = false;
    let sw_x = pins.gpio19.into_pull_up_input();
    let mut sw_x_flag = false;
    let sw_y = pins.gpio21.into_pull_up_input();
    let mut sw_y_flag = false;

    // Init UART
    // Set the USB bus
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set the serial port
    let mut serial = SerialPort::new(&usb_bus);

    // Set a USB device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    // Init SPI Driver
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let spi_1 = hal::Spi::<_, _, 8>::new(pac.SPI1);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi_1 = spi_1.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // Init GPIOs for LCD
    let mut backlight_pin = pins.gpio13.into_push_pull_output();
    let mut dc_select_pin = pins.gpio8.into_push_pull_output();

    // Main Loop
    loop {
        backlight_pin.set_high().unwrap();
        delay.delay_ms(500);
        backlight_pin.set_low().unwrap();
        // if sw_a.is_low().unwrap() {
        //     if sw_a_flag == true {
        //         continue;
        //     } else {
        //         let _ = serial.write(b"push sw_a\r\n");
        //         for _ in 0..3 {
        //             info!("on!");
        //             led_pin.set_high().unwrap();
        //             delay.delay_ms(200);
        //             info!("off!");
        //             led_pin.set_low().unwrap();
        //             delay.delay_ms(200);
        //         }
        //         sw_a_flag = true;
        //     }
        // } else if sw_b.is_low().unwrap() {
        //     if sw_b_flag == true {
        //         continue;
        //     } else {
        //         let _ = serial.write(b"push sw_b\r\n");
        //         for _ in 0..6 {
        //             info!("on!");
        //             led_pin.set_high().unwrap();
        //             delay.delay_ms(100);
        //             info!("off!");
        //             led_pin.set_low().unwrap();
        //             delay.delay_ms(100);
        //         }
        //         sw_b_flag = true;
        //     }
        // } else if sw_x.is_low().unwrap() {
        //     if sw_x_flag == true {
        //         continue;
        //     } else {
        //         let _ = serial.write(b"push sw_x\r\n");
        //         for _ in 0..12 {
        //             info!("on!");
        //             led_pin.set_high().unwrap();
        //             delay.delay_ms(50);
        //             info!("off!");
        //             led_pin.set_low().unwrap();
        //             delay.delay_ms(50);
        //         }
        //         sw_x_flag = true;
        //     }
        // } else if sw_y.is_low().unwrap() {
        //     if sw_y_flag == true {
        //         continue;
        //     } else {
        //         let _ = serial.write(b"push sw_y\r\n");
        //         for _ in 0..24 {
        //             info!("on!");
        //             led_pin.set_high().unwrap();
        //             delay.delay_ms(25);
        //             info!("off!");
        //             led_pin.set_low().unwrap();
        //             delay.delay_ms(25);
        //         }
        //         sw_y_flag = true;
        //     }
        // } else {
        //     sw_a_flag = false;
        //     sw_b_flag = false;
        //     sw_x_flag = false;
        //     sw_y_flag = false;
        //     led_pin.set_high().unwrap();
        //     let _ = usb_dev.poll(&mut [&mut serial]);
        //     delay.delay_ms(5);
        // }
    }
}

// End of file
