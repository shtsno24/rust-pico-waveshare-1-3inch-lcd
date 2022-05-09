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
const ST7789_1_30_INCH_HEIGHT: u16 = 240;
const ST7789_1_30_INCH_WIDTH: u16 = 240;
/* Control Registers and constant codes */
const ST7789_NOP: u8 = 0x00;
const ST7789_SWRESET: u8 = 0x01;
const ST7789_RDDID: u8 = 0x04;
const ST7789_RDDST: u8 = 0x09;

const ST7789_SLPIN: u8 = 0x10;
const ST7789_SLPOUT: u8 = 0x11;
const ST7789_PTLON: u8 = 0x12;
const ST7789_NORON: u8 = 0x13;

const ST7789_INVOFF: u8 = 0x20;
const ST7789_INVON: u8 = 0x21;
const ST7789_DISPOFF: u8 = 0x28;
const ST7789_DISPON: u8 = 0x29;
const ST7789_CASET: u8 = 0x2A;
const ST7789_RASET: u8 = 0x2B;
const ST7789_RAMWR: u8 = 0x2C;
const ST7789_RAMRD: u8 = 0x2E;

const ST7789_PTLAR: u8 = 0x30;
const ST7789_COLMOD: u8 = 0x3A;
const ST7789_MADCTL: u8 = 0x36;

// Memory Data Access Control Register (0x36H)
// MAP:     D7  D6  D5  D4  D3  D2  D1  D0
// param:   MY  MX  MV  ML  RGB MH  -   -

// Page Address Order ('0': Top to Bottom, '1': Bottom to Top) */
const ST7789_MADCTL_MY: u8 = 0x80;
// Column Address Order ('0': Left to Right, '1': Right to Left) */
const ST7789_MADCTL_MX: u8 = 0x40;
// Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
const ST7789_MADCTL_MV: u8 = 0x20;
// Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = LCD Refresh Bottom to Top) */
const ST7789_MADCTL_ML: u8 = 0x10;
// RGB/BGR Order ('0' = RGB, '1' = BGR) */
const ST7789_MADCTL_RGB: u8 = 0x00;

const ST7789_RDID1: u8 = 0xDA;
const ST7789_RDID2: u8 = 0xDB;
const ST7789_RDID3: u8 = 0xDC;
const ST7789_RDID4: u8 = 0xDD;

// Advanced options
const ST7789_COLOR_MODE_16BIT: u8 = 0x55; //  RGB565 (16bit)
const ST7789_COLOR_MODE_18BIT: u8 = 0x66; //  RGB666 (18bit)

// Color Codes(RGB565)
const COLOR_WHITE: u16 = 0xFFFF;
const COLOR_BLACK: u16 = 0x0000;
const COLOR_BLUE: u16 = 0x001F;
const COLOR_RED: u16 = 0xF800;
const COLOR_MAGENTA: u16 = 0xF81F;
const COLOR_GREEN: u16 = 0x07E0;
const COLOR_CYAN: u16 = 0x7FFF;
const COLOR_YELLOW: u16 = 0xFFE0;
const COLOR_GRAY: u16 = 0x8430;
const COLOR_BRED: u16 = 0xF81F;
const COLOR_GRED: u16 = 0xFFE0;
const COLOR_GBLUE: u16 = 0x07FF;
const COLOR_BROWN: u16 = 0xBC40;
const COLOR_BRRED: u16 = 0xFC07;
const COLOR_DARKBLUE: u16 = 0x01CF;
const COLOR_LIGHTBLUE: u16 = 0x7D7C;
const COLOR_GRAYBLUE: u16 = 0x5458;
const COLOR_LIGHTGREEN: u16 = 0x841F;
const COLOR_LGRAY: u16 = 0xC618;
const COLOR_LGRAYBLUE: u16 = 0xA651;
const COLOR_LBBLUE: u16 = 0x2B12;

#[entry]
fn main() -> ! {
    info!("Program start");

    // Buffer for NumToA
    let mut uart_buf = [0u8; 128];

    // User define variables
    let mut loop_cnt: u32 = 0;
    let lcd_rotate: u8 = 1;
    let mut ST7789_1_30_W_SHIFT: u16 = 0;
    let mut ST7789_1_30_H_SHIFT: u16 = 0;
    match lcd_rotate {
        0 => {
            ST7789_1_30_W_SHIFT = 0;
            ST7789_1_30_H_SHIFT = 80;
        }
        1 => {
            ST7789_1_30_W_SHIFT = 80;
            ST7789_1_30_H_SHIFT = 0;
        }
        2 => {
            ST7789_1_30_W_SHIFT = 0;
            ST7789_1_30_H_SHIFT = 0;
        }
        3 => {
            ST7789_1_30_W_SHIFT = 0;
            ST7789_1_30_H_SHIFT = 0;
        }
        _ => {}
    }

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
    // let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_csn = pins.gpio9.into_mode::<hal::gpio::FunctionSpi>();

    // Exchange the uninitialised SPI driver for an initialised one
    let spi_1 = hal::Spi::<_, _, 8>::new(pac.SPI1);
    let mut spi_1 = spi_1.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        50_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // Init GPIOs for LCD
    let mut reset_pin = pins.gpio12.into_push_pull_output();
    let mut backlight_pin = pins.gpio13.into_push_pull_output();
    let mut dc_select_pin = pins.gpio8.into_push_pull_output();

    // Init LCD
    backlight_pin.set_low().unwrap();
    // reset ST7789
    reset_pin.set_low().unwrap();
    delay.delay_ms(100);
    reset_pin.set_high().unwrap();
    delay.delay_ms(100);

    // set lotation
    let mut lcd_roatate_buffer: u8 = 0x00;
    match lcd_rotate {
        0 => lcd_roatate_buffer = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB,
        1 => lcd_roatate_buffer = ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB,
        2 => lcd_roatate_buffer = ST7789_MADCTL_RGB,
        3 => lcd_roatate_buffer = ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB,
        _ => {}
    }
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[ST7789_MADCTL]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[lcd_roatate_buffer]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);

    // config color mode
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[ST7789_COLMOD]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[ST7789_COLOR_MODE_16BIT]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);

    // porch control???
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xB2]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x0C, 0x0C, 0x00, 0x33, 0x33]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);

    // LCD Voltage Generator Settings
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xB7]).is_ok() { // gate control
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x35]).is_ok() { // default value
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xBB]).is_ok() { // VCOM settings
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x19]).is_ok() { // 0.725v (default 0.75v for 0x20)
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xC0]).is_ok() { // LCMCTRL
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x2C]).is_ok() { // default value
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xC2]).is_ok() { // VDV and VRH command Enable
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x01]).is_ok() { // default value
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xC3]).is_ok() { // VRH set
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x12]).is_ok() { // +-4.45v (defalut +-4.1v for 0x0B)
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xC4]).is_ok() { // VDV set
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x20]).is_ok() { // default value
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xC6]).is_ok() { // Frame rate control in normal mode
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0x0F]).is_ok() { // default value (60HZ)
         // SPI write was succesful
    }
    delay.delay_us(200);

    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xD0]).is_ok() { // Power control 1
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[0xA4, 0xA1]).is_ok() { // default value
         // SPI write was succesful
    }
    delay.delay_us(200);

    // Positive Voltage Gamma Control???
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xE0]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1
        .write(&[
            0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23,
        ])
        .is_ok()
    {
        // SPI write was succesful
    }
    delay.delay_us(200);

    // Negative Voltage Gamma Control???
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[0xE1]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1
        .write(&[
            0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23,
        ])
        .is_ok()
    {
        // SPI write was succesful
    }
    delay.delay_us(200);

    //Display Inversion On
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1
        .write(&[ST7789_INVON, ST7789_SLPOUT, ST7789_DISPON, ST7789_NORON])
        .is_ok()
    { //	Inversion ON
         // SPI write was succesful
    }
    delay.delay_ms(10);

    // Fill LCD in Light Blue
    // Set Address
    let w_start: u16 = ST7789_1_30_W_SHIFT;
    let h_start: u16 = ST7789_1_30_H_SHIFT;
    // let w_end: u16 = ST7789_1_30_INCH_WIDTH - 1 + ST7789_1_30_W_SHIFT;
    // let h_end: u16 = ST7789_1_30_INCH_HEIGHT - 1 + ST7789_1_30_H_SHIFT;
    let w_end: u16 = ST7789_1_30_INCH_WIDTH + ST7789_1_30_W_SHIFT;
    let h_end: u16 = ST7789_1_30_INCH_HEIGHT + ST7789_1_30_H_SHIFT;

    let fill_data_addr_w: [u8; 4] = [
        (w_start >> 8) as u8,
        (w_start & 0x00FF) as u8,
        (w_end >> 8) as u8,
        (w_end & 0x00FF) as u8,
    ];
    let fill_data_addr_h: [u8; 4] = [
        (h_start >> 8) as u8,
        (h_start & 0x00FF) as u8,
        (h_end >> 8) as u8,
        (h_end & 0x00FF) as u8,
    ];
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[ST7789_CASET]).is_ok() { // set colum(w, x) address
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&fill_data_addr_w).is_ok() { // send data
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[ST7789_RASET]).is_ok() { // set row(h, y) address
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&fill_data_addr_h).is_ok() { // send data
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[ST7789_RAMWR]).is_ok() { // Main screen turned on
         // SPI write was succesful
    }
    delay.delay_us(200);

    // Generate Data
    let fill_color_0: u16 = COLOR_RED;
    let fill_data_565_0: [u8; 2] = [(fill_color_0 >> 8) as u8, (fill_color_0 & 0x00FF) as u8];
    let fill_color_1: u16 = COLOR_BLUE;
    let fill_data_565_1: [u8; 2] = [(fill_color_1 >> 8) as u8, (fill_color_1 & 0x00FF) as u8];
    let fill_color_2: u16 = COLOR_GREEN;
    let fill_data_565_2: [u8; 2] = [(fill_color_2 >> 8) as u8, (fill_color_2 & 0x00FF) as u8];

    // Send Data
    dc_select_pin.set_high().unwrap(); // send data
    for w in w_start..w_end {
        for h in h_start..h_end {
            let fill_data_565 = if h % 3 == 0 && w % 3 == 0 {
                &fill_data_565_0[..]
            } else if h % 3 == 1 && w % 3 == 1 {
                &fill_data_565_1[..]
            } else {
                &fill_data_565_2[..]
            };
            if spi_1.write(fill_data_565).is_ok() { // Main screen turned on
                 // SPI write was succesful
            }
        }
    }
    backlight_pin.set_high().unwrap();

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
        if loop_cnt == 0 {
            // backlight_pin.set_high().unwrap();
            led_pin.set_high().unwrap();
        } else if loop_cnt == 100 {
            // backlight_pin.set_low().unwrap();
            led_pin.set_low().unwrap();
        } else if loop_cnt % 50 == 0 {
            delay.delay_ms(1);
            // write loop_cnt, then check the return
            // let _ = usb_dev.poll(&mut [&mut serial]);
            // let _ = serial.write(b"spi send_recv : send ");
            // let s = loop_cnt.numtoa(10, &mut uart_buf);
            // let _ = serial.write(s);
            // let _ = serial.write(b"\r\n");
            // let _ = serial.write(b"spi send_recv : recv ");
            // let send_success = spi_1.send(loop_cnt as u16);
            // match send_success {
            //     Ok(_) => {
            //         // We succeeded, check the read value
            //         if let Ok(_x) = spi_1.read() {
            //             let s = _x.numtoa(10, &mut uart_buf);
            //             let _ = serial.write(s);
            //             let _ = serial.write(b"\r\n\n");
            //             // We got back `x` in exchange for the 0x50 we sent.
            //         };
            //     }
            //     // Err(_) => todo!(),
            //     Err(_) => {}
            // }

            // // Do a read+write at the same time. Data in `buffer` will be replaced with
            // // the data read from the SPI device.
            // const LEN_SPI_BUFF_0: usize = 3;
            // let mut buffer: [u16; LEN_SPI_BUFF_0] = [11 * (loop_cnt as u16), 2020, 3300];
            // let _ = usb_dev.poll(&mut [&mut serial]);
            // let _ = serial.write(b"spi transfer_0 : send [");
            // for _i in 0..LEN_SPI_BUFF_0 {
            //     let s = (buffer[_i]).numtoa(10, &mut uart_buf);
            //     let _ = serial.write(s);
            //     if _i < (LEN_SPI_BUFF_0 - 1) {
            //         let _ = serial.write(b", ");
            //     }
            // }
            // let _ = serial.write(b"]\r\n");
            // let _ = serial.write(b"spi transfer_0 : recv ");
            // let transfer_success = spi_1.transfer(&mut buffer);
            // let _ = serial.write(b"[");
            // for _d in transfer_success.iter() {
            //     for _i in 0..LEN_SPI_BUFF_0 {
            //         let s = (_d[_i]).numtoa(10, &mut uart_buf);
            //         let _ = serial.write(s);
            //         if _i < (LEN_SPI_BUFF_0 - 1) {
            //             let _ = serial.write(b", ");
            //         }
            //     }
            // }
            // let _ = serial.write(b"]\r\n");

            // #[allow(clippy::single_match)]
            // match transfer_success {
            //     Ok(_) => {}  // Handle success
            //     Err(_) => {} // handle errors
            // };

            // const LEN_SPI_BUFF_1: usize = 4;
            // let mut buffer: [u16; LEN_SPI_BUFF_1] = [32 * (loop_cnt as u16), 2048, 4096, 8192];
            // let _ = usb_dev.poll(&mut [&mut serial]);
            // let _ = serial.write(b"spi transfer_1 : send [");
            // for _i in 0..LEN_SPI_BUFF_1 {
            //     let s = (buffer[_i]).numtoa(10, &mut uart_buf);
            //     let _ = serial.write(s);
            //     if _i < (LEN_SPI_BUFF_1 - 1) {
            //         let _ = serial.write(b", ");
            //     }
            // }
            // let _ = serial.write(b"]\r\n");
            // let _ = serial.write(b"spi transfer_1 : recv ");
            // let transfer_success = spi_1.transfer(&mut buffer);
            // let _ = serial.write(b"[");
            // for _d in transfer_success.iter() {
            //     for _i in 0..LEN_SPI_BUFF_1 {
            //         let s = (_d[_i]).numtoa(10, &mut uart_buf);
            //         let _ = serial.write(s);
            //         if _i < (LEN_SPI_BUFF_1 - 1) {
            //             let _ = serial.write(b", ");
            //         }
            //     }
            // }
            // let _ = serial.write(b"]\r\n\n==========\r\n\n");

            // #[allow(clippy::single_match)]
            // match transfer_success {
            //     Ok(_) => {}  // Handle success
            //     Err(_) => {} // handle errors
            // };
        }
        let _ = usb_dev.poll(&mut [&mut serial]);
        delay.delay_ms(10);
        loop_cnt += 1;
        if loop_cnt > 200 {
            loop_cnt = 0;
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
        // }
    }
}

// End of file
