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
/* Commands for Control Registers*/
pub enum St7789Command {
    Nop = 0x00,
    SwReset = 0x01,
    RddID = 0x04,
    RddST = 0x09,
    SLPin = 0x10,
    SLPout = 0x11,
    PTLOn = 0x12,
    NOROn = 0x13,
    InvOff = 0x20,
    InvOn = 0x21,
    DispOff = 0x28,
    DispOn = 0x29,
    CASet = 0x2A,
    RASet = 0x2B,
    RamWr = 0x2C,
    RamRd = 0x2E,
    PTLAr = 0x30,
    COLMod = 0x3A,
    MADCtl = 0x36,
    RdID1 = 0xDA,
    RdID2 = 0xDB,
    RdID3 = 0xDC,
    RdID4 = 0xDD,
    // Advanced options
    ColorMode16bit = 0x55, //  RGB565 (16bit)
    ColorMode18bit = 0x66, //  RGB666 (18bit)
}
impl Into<u8> for St7789Command {
    fn into(self) -> u8 {
        self as u8
    }
}

/* Commands for Memory Data Access Control Register */
pub enum ST7789MADCtlCommand {
    // Memory Data Access Control Register (0x36H)
    // MAP:     D7  D6  D5  D4  D3  D2  D1  D0
    // param:   MY  MX  MV  ML  RGB MH  -   -

    // Page Address Order ('0': Top to Bottom, '1': Bottom to Top) */
    MADCtlMY = 0x80,
    // Column Address Order ('0': Left to Right, '1': Right to Left) */
    MADCtlMX = 0x40,
    // Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
    MADCtlMV = 0x20,
    // Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = LCD Refresh Bottom to Top) */
    MADCtlML = 0x10,
    // RGB/BGR Order ('0' = RGB, '1' = BGR) */
    MADCtlRGB = 0x00,
}
impl Into<u8> for ST7789MADCtlCommand {
    fn into(self) -> u8 {
        self as u8
    }
}

// Color Codes(RGB565)
pub enum DisplayColors {
    ColorWhite = 0xFFFF,
    ColorBlack = 0x0000,
    ColorBlue = 0x001F,
    ColorRed = 0xF800,
    ColorMagenta = 0xF81F,
    ColorGreen = 0x07E0,
    ColorCyan = 0x7FFF,
    ColorYellow = 0xFFE0,
    ColorGray = 0x8430,
    ColorGBlue = 0x07FF,
    ColorBrown = 0xBC40,
    ColorBread = 0xFC07,
    ColorDarkBlue = 0x01CF,
    ColorLightBlue = 0x7D7C,
    ColorGrayBlue = 0x5458,
    ColorLightGreen = 0x841F,
    ColorLGray = 0xC618,
    ColorLGrayBlue = 0xA651,
    ColorLbBlue = 0x2B12,
}
impl Into<u16> for DisplayColors {
    fn into(self) -> u16 {
        self as u16
    }
}

// fn st7789_write(
//     spi: Spi<Enabled, SPI1, 8>,
//     dc_sel_pin: Pin,
//     command: &mut [u8],
//     data: &mut [u8],
// ) -> () {
//     dc_sel_pin.set_low().unwrap(); // send command
//     if spi.write(command).is_ok() {
//         // SPI write was succesful
//     }
//     delay.delay_us(200);
//     dc_sel_pin.set_high().unwrap(); // send data
//     if spi.write(data).is_ok() {
//         // SPI write was succesful
//     }
//     delay.delay_us(200);
// }

#[entry]
fn main() -> ! {
    info!("Program start");

    // Buffer for NumToA
    let mut uart_buf = [0u8; 128];

    // User define variables
    let mut loop_cnt: u32 = 0;
    let lcd_rotate: u8 = 1;
    let mut st7789_1_30_w_shift: u16 = 0;
    let mut st7789_1_30_h_shift: u16 = 0;
    match lcd_rotate {
        0 => {
            st7789_1_30_w_shift = 0;
            st7789_1_30_h_shift = 80;
        }
        1 => {
            st7789_1_30_w_shift = 80;
            st7789_1_30_h_shift = 0;
        }
        2 => {
            st7789_1_30_w_shift = 0;
            st7789_1_30_h_shift = 0;
        }
        3 => {
            st7789_1_30_w_shift = 0;
            st7789_1_30_h_shift = 0;
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
    let mut lcd_rotate_buffer: u8 = 0x00;
    match lcd_rotate {
        0 => {
            lcd_rotate_buffer = ST7789MADCtlCommand::MADCtlMX as u8
                | ST7789MADCtlCommand::MADCtlMY as u8
                | ST7789MADCtlCommand::MADCtlRGB as u8
        }
        1 => {
            lcd_rotate_buffer = ST7789MADCtlCommand::MADCtlMY as u8
                | ST7789MADCtlCommand::MADCtlMV as u8
                | ST7789MADCtlCommand::MADCtlRGB as u8
        }
        2 => lcd_rotate_buffer = ST7789MADCtlCommand::MADCtlRGB as u8,
        3 => {
            lcd_rotate_buffer = ST7789MADCtlCommand::MADCtlMX as u8
                | ST7789MADCtlCommand::MADCtlMV as u8
                | ST7789MADCtlCommand::MADCtlRGB as u8
        }
        _ => {}
    }
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[St7789Command::MADCtl as u8]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[lcd_rotate_buffer]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);

    // config color mode
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[St7789Command::COLMod as u8]).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&[St7789Command::ColorMode16bit as u8]).is_ok() {
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
        .write(&[
            St7789Command::InvOn as u8,
            St7789Command::SLPout as u8,
            St7789Command::DispOn as u8,
            St7789Command::NOROn as u8,
        ])
        .is_ok()
    { //	Inversion ON
         // SPI write was succesful
    }
    delay.delay_ms(10);

    // Fill LCD in Light Blue
    // Set Address
    let w_start: u16 = st7789_1_30_w_shift;
    let h_start: u16 = st7789_1_30_h_shift;
    // let w_end: u16 = ST7789_1_30_INCH_WIDTH - 1 + st7789_1_30_w_shift;
    // let h_end: u16 = ST7789_1_30_INCH_HEIGHT - 1 + st7789_1_30_h_shift;
    let w_end: u16 = ST7789_1_30_INCH_WIDTH + st7789_1_30_w_shift;
    let h_end: u16 = ST7789_1_30_INCH_HEIGHT + st7789_1_30_h_shift;
    let w_mid: u16 = (w_end + w_start) / 2;
    let h_mid: u16 = (h_end + h_start) / 2;

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
    if spi_1.write(&[St7789Command::CASet as u8]).is_ok() { // set colum(w, x) address
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&fill_data_addr_w).is_ok() { // send data
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[St7789Command::RASet as u8]).is_ok() { // set row(h, y) address
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_high().unwrap(); // send data
    if spi_1.write(&fill_data_addr_h).is_ok() { // send data
         // SPI write was succesful
    }
    delay.delay_us(200);
    dc_select_pin.set_low().unwrap(); // send command
    if spi_1.write(&[St7789Command::RamWr as u8]).is_ok() { // Main screen turned on
         // SPI write was succesful
    }
    delay.delay_us(200);

    // Generate Data
    let fill_color_0: u16 = DisplayColors::ColorRed as u16;
    let fill_data_565_0: [u8; 2] = [(fill_color_0 >> 8) as u8, (fill_color_0 & 0x00FF) as u8];
    let fill_color_1: u16 = DisplayColors::ColorBlue as u16;
    let fill_data_565_1: [u8; 2] = [(fill_color_1 >> 8) as u8, (fill_color_1 & 0x00FF) as u8];
    let fill_color_2: u16 = DisplayColors::ColorGreen as u16;
    let fill_data_565_2: [u8; 2] = [(fill_color_2 >> 8) as u8, (fill_color_2 & 0x00FF) as u8];

    // Send Data
    dc_select_pin.set_high().unwrap(); // send data
    for w in w_start..w_end {
        for h in h_start..h_end {
            let fill_data_565 = if h <= h_mid && w <= w_mid {
                &fill_data_565_0[..]
            } else if h > h_mid && w > w_mid {
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
