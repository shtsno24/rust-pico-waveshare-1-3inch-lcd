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

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use bsp::hal;
use bsp::hal::pac;
// use hal::pac as pac;
// use rp2040_hal as hal;
// Pull in any important traits
// use bsp::hal::prelude::*;
use cortex_m::prelude::*;

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

// ST7789 Constants
const ST7789_1_30_INCH_HEIGHT: u32 = 240;
const ST7789_1_30_INCH_WIDTH: u32 = 240;
const ST7789_1_30_X_SHIFT: u32 = 0;
const ST7789_1_30_Y_SHIFT: u32 = 80;
/* Control Registers and constant codes */
const ST7789_NOP: u32 = 0x00;
const ST7789_SWRESET: u32 = 0x01;
const ST7789_RDDID: u32 = 0x04;
const ST7789_RDDST: u32 = 0x09;

const ST7789_SLPIN: u32 = 0x10;
const ST7789_SLPOUT: u32 = 0x11;
const ST7789_PTLON: u32 = 0x12;
const ST7789_NORON: u32 = 0x13;

const ST7789_INVOFF: u32 = 0x20;
const ST7789_INVON: u32 = 0x21;
const ST7789_DISPOFF: u32 = 0x28;
const ST7789_DISPON: u32 = 0x29;
const ST7789_CASET: u32 = 0x2A;
const ST7789_RASET: u32 = 0x2B;
const ST7789_RAMWR: u32 = 0x2C;
const ST7789_RAMRD: u32 = 0x2E;

const ST7789_PTLAR: u32 = 0x30;
const ST7789_COLMOD: u32 = 0x3A;
const ST7789_MADCTL: u32 = 0x36;

// Memory Data Access Control Register (0x36H)
// MAP:     D7  D6  D5  D4  D3  D2  D1  D0
// param:   MY  MX  MV  ML  RGB MH  -   -

// Page Address Order ('0': Top to Bottom, '1': Bottom to Top) */
const ST7789_MADCTL_MY: u32 = 0x80;
// Column Address Order ('0': Left to Right, '1': Right to Left) */
const ST7789_MADCTL_MX: u32 = 0x40;
// Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
const ST7789_MADCTL_MV: u32 = 0x20;
// Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = LCD Refresh Bottom to Top) */
const ST7789_MADCTL_ML: u32 = 0x10;
// RGB/BGR Order ('0' = RGB, '1' = BGR) */
const ST7789_MADCTL_RGB: u32 = 0x00;

const ST7789_RDID1: u32 = 0xDA;
const ST7789_RDID2: u32 = 0xDB;
const ST7789_RDID3: u32 = 0xDC;
const ST7789_RDID4: u32 = 0xDD;

// Advanced options
const ST7789_COLOR_MODE_16BIT: u32 = 0x55; //  RGB565 (16bit)
const ST7789_COLOR_MODE_18BIT: u32 = 0x66; //  RGB666 (18bit)

// Color Codes(RGB565)
const COLOR_WHITE: u32 = 0xFFFF;
const COLOR_BLACK: u32 = 0x0000;
const COLOR_BLUE: u32 = 0x001F;
const COLOR_RED: u32 = 0xF800;
const COLOR_MAGENTA: u32 = 0xF81F;
const COLOR_GREEN: u32 = 0x07E0;
const COLOR_CYAN: u32 = 0x7FFF;
const COLOR_YELLOW: u32 = 0xFFE0;
const COLOR_GRAY: u32 = 0x8430;
const COLOR_BRED: u32 = 0xF81F;
const COLOR_GRED: u32 = 0xFFE0;
const COLOR_GBLUE: u32 = 0x07FF;
const COLOR_BROWN: u32 = 0xBC40;
const COLOR_BRRED: u32 = 0xFC07;
const COLOR_DARKBLUE: u32 = 0x01CF;
const COLOR_LIGHTBLUE: u32 = 0x7D7C;
const COLOR_GRAYBLUE: u32 = 0x5458;
const COLOR_LIGHTGREEN: u32 = 0x841F;
const COLOR_LGRAY: u32 = 0xC618;
const COLOR_LGRAYBLUE: u32 = 0xA651;
const COLOR_LBBLUE: u32 = 0x2B12;

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

    // Init SPI Driver
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let spi_1 = hal::Spi::<_, _, 16>::new(pac.SPI1);

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

    let mut loop_cnt: u32 = 0;

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

    // Main Loop
    loop {
        if loop_cnt == 1003 {
            // Init LCDs
            // Write out 0, ignore return value
            let _ = usb_dev.poll(&mut [&mut serial]);
            let _ = serial.write(b"start spi send\r\n");
            if spi_1.write(&[0]).is_ok() {
                // SPI write was succesful
                let _ = serial.write(b"spi send done\r\n");
            } else {
                let _ = serial.write(b"spi send undone\r\n");
            };

            // write 50, then check the return
            let _ = usb_dev.poll(&mut [&mut serial]);
            let send_success = spi_1.send(1024);
            match send_success {
                Ok(_) => {
                    // We succeeded, check the read value
                    if let Ok(_x) = spi_1.read() {
                        // We got back `x` in exchange for the 0x50 we sent.
                        let _ = serial.write(b"matched\r\n");
                    };
                }
                // Err(_) => todo!(),
                Err(_) => {
                    let _ = serial.write(b"unmatched\r\n");
                }
            }

            // Do a read+write at the same time. Data in `buffer` will be replaced with
            // the data read from the SPI device.
            let mut buffer: [u16; 4] = [1024, 2048, 4096, 8192];
            let transfer_success = spi_1.transfer(&mut buffer);
            #[allow(clippy::single_match)]
            match transfer_success {
                Ok(_) => {}  // Handle success
                Err(_) => {} // handle errors
            };
        }

        if (loop_cnt % 200) % 2 == 0 {
            backlight_pin.set_high().unwrap();
            led_pin.set_high().unwrap();
        } else if (loop_cnt % 200) % 2 == 1 {
            backlight_pin.set_low().unwrap();
            led_pin.set_low().unwrap();
        }
        let _ = usb_dev.poll(&mut [&mut serial]);
        delay.delay_ms(5);
        loop_cnt += 1;
        if loop_cnt > 1000 {
            loop_cnt = 1;
        }
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
