// see https://github.com/Floyd-Fish/ST7789-STM32
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
use bsp::hal::pac;
use bsp::{hal, XOSC_CRYSTAL_FREQ};
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
// const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const XTAL_FREQ_HZ: u32 = XOSC_CRYSTAL_FREQ;

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

fn st7789_write_com_data(
    spi: &mut hal::Spi<hal::spi::Enabled, hal::pac::SPI1, 8>,
    delay: &mut cortex_m::delay::Delay,
    dc_sel_pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    command: &[u8],
    data: &[u8],
) -> () {
    // see https://github.com/rp-rs/rp-hal/blob/main/boards/rp-pico/examples/pico_spi_sd_card.rs
    dc_sel_pin.set_low().unwrap(); // send command
    if spi.write(command).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
    dc_sel_pin.set_high().unwrap(); // send data
    if spi.write(data).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
}

fn st7789_write_command(
    spi: &mut hal::Spi<hal::spi::Enabled, hal::pac::SPI1, 8>,
    delay: &mut cortex_m::delay::Delay,
    dc_sel_pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    command: &[u8],
) -> () {
    // see https://github.com/rp-rs/rp-hal/blob/main/boards/rp-pico/examples/pico_spi_sd_card.rs
    dc_sel_pin.set_low().unwrap(); // send command
    if spi.write(command).is_ok() {
        // SPI write was succesful
    }
    delay.delay_us(200);
}

fn st7789_write_frame(
    spi: &mut hal::Spi<hal::spi::Enabled, hal::pac::SPI1, 8>,
    dc_sel_pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    fbuffer: &[[u8; 2]],
) -> () {
    // see https://github.com/rp-rs/rp-hal/blob/main/boards/rp-pico/examples/pico_spi_sd_card.rs
    dc_sel_pin.set_high().unwrap(); // send data
    for hw in 0..fbuffer.len() {
        if spi.write(&fbuffer[hw]).is_ok() {
            // SPI write was succesful
        }
    }
}

fn fill_frame_quad_power(
    color_565_0: [u8; 2],
    color_565_1: [u8; 2],
    color_565_2: [u8; 2],
    color_565_3: [u8; 2],
) -> [[u8; 2]; (ST7789_1_30_INCH_HEIGHT * ST7789_1_30_INCH_WIDTH) as usize] {
    let mut cnt_frame: usize = 0;
    let mut fbuffer: [[u8; 2]; (ST7789_1_30_INCH_WIDTH * ST7789_1_30_INCH_HEIGHT) as usize] =
        [[0, 0]; (ST7789_1_30_INCH_WIDTH * ST7789_1_30_INCH_HEIGHT) as usize];
    for w in (0..ST7789_1_30_INCH_WIDTH).rev() {
        for h in 0..ST7789_1_30_INCH_HEIGHT {
            let fill_data_565 =
                if h <= (ST7789_1_30_INCH_HEIGHT / 2) && w <= (ST7789_1_30_INCH_WIDTH / 2) {
                    &color_565_0[..]
                } else if h <= (ST7789_1_30_INCH_HEIGHT / 2) && w > (ST7789_1_30_INCH_WIDTH / 2) {
                    &color_565_1[..]
                } else if h > (ST7789_1_30_INCH_HEIGHT / 2) && w <= (ST7789_1_30_INCH_WIDTH / 2) {
                    &color_565_2[..]
                } else {
                    &color_565_3[..]
                };
            fbuffer[cnt_frame][0] = fill_data_565[0];
            fbuffer[cnt_frame][1] = fill_data_565[1];
            cnt_frame += 1;
        }
    }
    fbuffer
}

#[entry]
fn main() -> ! {
    info!("Program start");

    // Buffer for NumToA
    let mut uart_buf = [0u8; 128];

    // User define variables
    let mut loop_cnt: u32 = 0;
    let lcd_rotate: u8 = 0;
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Set the pins to their default state
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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

    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[St7789Command::MADCtl as u8],
        &[lcd_rotate_buffer],
    );

    // config color mode
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[St7789Command::COLMod as u8],
        &[St7789Command::ColorMode16bit as u8],
    );

    // porch control???
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[0xB2],
        &[0x0C, 0x0C, 0x00, 0x33, 0x33],
    );

    // LCD Voltage Generator Settings
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xB7], &[0x35]);

    // VCOM settings
    // 0.725v (default 0.75v for 0x20)
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xBB], &[0x19]);

    // LCMCTRL
    // default value
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xC0], &[0x2C]);

    // VDV and VRH command Enable
    // default value
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xC2], &[0x01]);

    // VRH set
    // +-4.45v (defalut +-4.1v for 0x0B)
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xC3], &[0x12]);

    // VDV set
    // default value
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xC4], &[0x20]);

    // Frame rate control in normal mode
    // default value (60HZ)
    st7789_write_com_data(&mut spi_1, &mut delay, &mut dc_select_pin, &[0xC6], &[0x0F]);

    // Power control 1
    // default value
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[0xD0],
        &[0xA4, 0xA1],
    );

    // Positive Voltage Gamma Control???
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[0xE0],
        &[
            0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23,
        ],
    );

    // Negative Voltage Gamma Control???
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[0xE1],
        &[
            0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23,
        ],
    );

    //Display Inversion On
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[
            St7789Command::InvOn as u8,
            St7789Command::SLPout as u8,
            St7789Command::DispOn as u8,
            St7789Command::NOROn as u8,
        ],
        &[0x00],
    );

    // init frame
    // set colum(w, x) address
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[St7789Command::CASet as u8],
        &fill_data_addr_w,
    );

    // set row(h, y) address
    st7789_write_com_data(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[St7789Command::RASet as u8],
        &fill_data_addr_h,
    );

    // Main screen turned on
    st7789_write_command(
        &mut spi_1,
        &mut delay,
        &mut dc_select_pin,
        &[St7789Command::RamWr as u8],
    );

    // Generate Data
    let fill_color_r: u16 = DisplayColors::ColorRed as u16;
    let fill_data_565_r: [u8; 2] = [(fill_color_r >> 8) as u8, (fill_color_r & 0x00FF) as u8];
    let fill_color_b: u16 = DisplayColors::ColorBlue as u16;
    let fill_data_565_b: [u8; 2] = [(fill_color_b >> 8) as u8, (fill_color_b & 0x00FF) as u8];
    let fill_color_g: u16 = DisplayColors::ColorGreen as u16;
    let fill_data_565_g: [u8; 2] = [(fill_color_g >> 8) as u8, (fill_color_g & 0x00FF) as u8];

    // Send Frame Data
    let frame_buffer = fill_frame_quad_power(
        fill_data_565_g,
        fill_data_565_r,
        fill_data_565_b,
        fill_data_565_g,
    );
    st7789_write_frame(&mut spi_1, &mut dc_select_pin, &frame_buffer);
    backlight_pin.set_high().unwrap(); // Turn On BackLights

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
            let frame_buffer = fill_frame_quad_power(
                fill_data_565_r,
                fill_data_565_b,
                fill_data_565_g,
                fill_data_565_g,
            );
            st7789_write_frame(&mut spi_1, &mut dc_select_pin, &frame_buffer);
            led_pin.set_high().unwrap();
        } else if loop_cnt == 100 {
            let frame_buffer = fill_frame_quad_power(
                fill_data_565_g,
                fill_data_565_r,
                fill_data_565_b,
                fill_data_565_g,
            );
            st7789_write_frame(&mut spi_1, &mut dc_select_pin, &frame_buffer);
            led_pin.set_low().unwrap();
        } else if loop_cnt == 150 {
            let frame_buffer = fill_frame_quad_power(
                fill_data_565_g,
                fill_data_565_g,
                fill_data_565_r,
                fill_data_565_b,
            );
            st7789_write_frame(&mut spi_1, &mut dc_select_pin, &frame_buffer);
            led_pin.set_high().unwrap();
        } else if loop_cnt == 200 {
            let frame_buffer = fill_frame_quad_power(
                fill_data_565_b,
                fill_data_565_g,
                fill_data_565_g,
                fill_data_565_r,
            );
            st7789_write_frame(&mut spi_1, &mut dc_select_pin, &frame_buffer);
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
        }
        let _ = usb_dev.poll(&mut [&mut serial]);
        delay.delay_ms(10);
        loop_cnt += 1;
        if loop_cnt > 250 {
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
