//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{dma, dma::DMAExt},
};
// use defmt::*;
use defmt_rtt as _;
// use embedded_hal::{blocking::spi::transfer, digital::v2::OutputPin as OutputPinV2};
use embedded_hal_bus::spi::NoDelay;
use embedded_hal_new::{
    digital::{Error as DigitalErrorNew, OutputPin as OutputPinNew},
    spi::{ErrorType as SpiErrorTypeNew, SpiBus as SpiBusNew},
};
use heatshrink::decoder::HeatshrinkDecoder;
use panic_probe as _;

// use shared_pin::SharedPin;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;
use waveshare_rp2040_lcd_0_96 as bsp;

use bsp::hal::{self, clocks::Clock, gpio, pac, sio::Sio, spi};
// use embedded_hal::i2c::{I2c, Operation};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Used to demonstrate writing formatted strings
use core::{cell::RefCell, convert::Infallible, fmt::Write};
use heapless::String;

use cortex_m::singleton;

// Embed the `Hz` function/trait:
use fugit::RateExtU32;

// use embedded_graphics::{
//     image::{Image, ImageRaw},
//     pixelcolor::{raw::LittleEndian, Rgb565},
//     prelude::{Point, RgbColor, Size},
//     primitives::{Circle, Primitive, PrimitiveStyleBuilder, Rectangle},
//     Drawable,
// };
use gc9a01::{command::Command, prelude::*, Gc9a01, SPIDisplayInterface};
// use qmi8658::Qmi8658;

mod apple;

// type BoxedDisplayDriver<'a> = Gc9a01<
//     SPIInterface<spi::SpiDevice, embedded_hal::digital::v2::OutputPin>,
//     DisplayResolution240x240,
//     BufferedGraphics<DisplayResolution240x240>,
// >;

// use include_bytes_plus::include_bytes;

// const DATA_0: [u16; 57600] = include_bytes!("./assets/dance/danc_00000.png.raw" as u16);
// const DATA_1: [u16; 57600] = include_bytes!("./assets/dance/danc_00010.png.raw" as u16);
// const DATA_2: [u16; 57600] = include_bytes!("./assets/dance/danc_00020.png.raw" as u16);
// const DATA_3: [u16; 57600] = include_bytes!("./assets/dance/danc_00030.png.raw" as u16);
// const DATA_4: [u16; 57600] = include_bytes!("./assets/dance/danc_00040.png.raw" as u16);
// const DATA_5: [u16; 57600] = include_bytes!("./assets/dance/danc_00050.png.raw" as u16);
// const DATA_6: [u16; 57600] = include_bytes!("./assets/dance/danc_00060.png.raw" as u16);
// const DATA_7: [u16; 57600] = include_bytes!("./assets/dance/danc_00070.png.raw" as u16);
// const DATA_8: [u16; 57600] = include_bytes!("./assets/dance/danc_00080.png.raw" as u16);
// const DATA_9: [u16; 57600] = include_bytes!("./assets/dance/danc_00090.png.raw" as u16);
// const DATA_10: [u16; 57600] = include_bytes!("./assets/dance/danc_00100.png.raw" as u16);
// const DATA_11: [u16; 57600] = include_bytes!("./assets/dance/danc_00110.png.raw" as u16);
// const DATA_12: [u16; 57600] = include_bytes!("./assets/dance/danc_00120.png.raw" as u16);
// const DATA_13: [u16; 57600] = include_bytes!("./assets/dance/danc_00130.png.raw" as u16);

// const ALL_DATA: [[u16; 57600]; 14] = [
//     DATA_0, DATA_1, DATA_2, DATA_3, DATA_4, DATA_5, DATA_6, DATA_7, DATA_8, DATA_9, DATA_10,
//     DATA_11, DATA_12, DATA_13,
// ];

// const APPLE_DATA: &[u8; 1997640] = include_bytes!("../workspace/frames_bundle.bin");

const APPLE_FRAME_WIDTH: u16 = 240;
const APPLE_FRAME_HEIGHT: u16 = 179;
const APPLE_FRAME_BITS: usize = APPLE_FRAME_WIDTH as usize * APPLE_FRAME_HEIGHT as usize;
const APPLE_FRAME_BYTES: usize = APPLE_FRAME_BITS / 8;

// const APPLE_FRAME_COUNT: usize = APPLE_DATA.len() / APPLE_FRAME_BYTES;

const DISPLAY_BUFFER_SIZE: usize = 240 * 240 * 2;
const DISPLAY_BUFFER_OFFSET: usize = 240 * 30 * 2;

// const DATA_0: &[u8; 115200] = include_bytes!("../assets/spiral/spiral_01.raw");
// const DATA_1: &[u8] = include_bytes!("../assets/spiral/spiral_02.raw");
// const DATA_2: &[u8] = include_bytes!("../assets/spiral/spiral_03.raw");
// const DATA_3: &[u8] = include_bytes!("../assets/spiral/spiral_04.raw");
// const DATA_4: &[u8] = include_bytes!("../assets/spiral/spiral_05.raw");
// const DATA_5: &[u8] = include_bytes!("../assets/spiral/spiral_06.raw");
// const DATA_6: &[u8] = include_bytes!("../assets/spiral/spiral_07.raw");
// const DATA_7: &[u8] = include_bytes!("../assets/spiral/spiral_08.raw");
// const DATA_8: &[u8] = include_bytes!("../assets/spiral/spiral_09.raw");
// const DATA_9: &[u8] = include_bytes!("../assets/spiral/spiral_10.raw");
// const DATA_10: &[u8] = include_bytes!("../assets/spiral/spiral_11.raw");

// const ALL_DATA: [&[u8]; 11] = [
//     DATA_0, DATA_1, DATA_2, DATA_3, DATA_4, DATA_5, DATA_6, DATA_7, DATA_8, DATA_9, DATA_10,
// ];

struct MySpiWrapper<BUS> {
    spi: Option<BUS>,
}

impl<BUS> MySpiWrapper<BUS> {
    pub fn new(lcd_spi_bus: BUS) -> Self {
        Self {
            spi: Some(lcd_spi_bus),
        }
    }
    pub fn new_empty() -> Self {
        Self { spi: None }
    }
}

impl<BUS> SpiErrorTypeNew for MySpiWrapper<BUS>
where
    BUS: SpiBusNew,
    <BUS as SpiErrorTypeNew>::Error: DigitalErrorNew,
{
    type Error = BUS::Error;
}

impl<BUS> SpiBusNew for MySpiWrapper<BUS>
where
    BUS: SpiBusNew,
    <BUS as SpiErrorTypeNew>::Error: DigitalErrorNew,
{
    fn write(&mut self, data: &[u8]) -> Result<(), BUS::Error> {
        self.spi.as_mut().unwrap().write(data)
    }
    fn read(&mut self, data: &mut [u8]) -> Result<(), BUS::Error> {
        self.spi.as_mut().unwrap().read(data)
    }
    fn transfer<'w>(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.spi.as_mut().unwrap().transfer(read, write)
    }
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.as_mut().unwrap().transfer_in_place(words)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.spi.as_mut().unwrap().flush()
    }
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();

    //--------------------------
    // Timing Initialization
    //--------------------------

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
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

    // Timer
    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Delay
    // let sys_freq = clocks.system_clock.freq().to_Hz();
    // let mut delay = Delay::new(core.SYST, sys_freq);

    //--------------------------
    // USB Serial Initialization
    //--------------------------

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2e8a, 0x000A))
        .strings(&[StringDescriptors::default()
            .manufacturer("Raspberry Pi")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut heatshrink = HeatshrinkDecoder::new();

    heatshrink.reset();

    // Wait for the host to connect and send something
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                    break;
                }
            }
        }
        if timer.get_counter().ticks() > 2_000_000 {
            break;
        }
    }
    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Welcome!\r\n");

    let clk_ref = clocks.reference_clock.freq().to_MHz();
    let clk_sys = clocks.system_clock.freq().to_MHz();
    let clk_peri = clocks.peripheral_clock.freq().to_MHz();

    let mut text: String<128> = String::new();
    write!(
        &mut text,
        "Reference clock: {}MHz, System clock: {}MHz, Peripheral clock: {}MHz\r\n",
        clk_ref, clk_sys, clk_peri
    )
    .unwrap();
    write_and_poll_usb_once(&mut usb_dev, &mut serial, text.as_bytes());

    //--------------------------
    // Pin configuration
    //--------------------------
    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Configuring pins\r\n");

    // The single-cycle I/O block controls our GPIO pins
    let sio = Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LCD SPI pins
    let pin_lcd_spi_cs = pins.gp9.into_push_pull_output();
    let pin_lcd_spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gp10.reconfigure();
    let pin_lcd_spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gp11.reconfigure();
    let pin_lcd_spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gp12.reconfigure();

    // Other LCD control pins
    let pin_lcd_dc = pins.gp8.into_push_pull_output();
    let mut pin_lcd_backlight = pins.gp25.into_push_pull_output();
    let mut pin_lcd_reset = pins.gp13.into_push_pull_output();

    // IMU I2C pins
    // let pin_imu_i2c_sda = pins.gp23;
    // let pin_imu_i2c_scl = pins.gp24;

    //--------------------------
    // DMA Shenanigans
    //--------------------------

    // Get the DMA channels
    let dma = pac.DMA.split(&mut pac.RESETS);
    let mut dma_spi_channel = Some(dma.ch0);
    // let dma_spi_

    //--------------------------
    // LCD Initialization
    //--------------------------

    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Initializing SPI\r\n");

    // Create the SPI bus instance for the SPI0 device using the led pins
    let lcd_spi_bus = spi::Spi::<_, _, _, 8>::new(
        pac.SPI1,
        (pin_lcd_spi_mosi, pin_lcd_spi_miso, pin_lcd_spi_sclk),
    );

    // Exchange the uninitialised SPI bus for an initialised one
    let mut lcd_spi_bus = lcd_spi_bus.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        63.MHz(), // Max baud rate for the SPI bus is 62.5MHz (half the peripheral clock frequency of 125MHz)
        embedded_hal::spi::MODE_0,
    );

    // Re-set the baud so we can log what it was actually set to
    let actual_baud = lcd_spi_bus.set_baudrate(clocks.peripheral_clock.freq(), 125.MHz());
    let mut text: String<64> = String::new();
    write!(&mut text, "SPI baud: {}MHz\r\n", actual_baud.to_MHz()).unwrap();
    write_and_poll_usb_once(&mut usb_dev, &mut serial, text.as_bytes());

    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Initializing display\r\n");
    let lcd_spi_bus = MySpiWrapper::new(lcd_spi_bus);

    // Wrap the SPI bus and pins in a RefCell so we can share them with the display driver
    let lcd_spi_bus = RefCell::new(lcd_spi_bus);
    let pin_lcd_spi_cs = RefCell::new(pin_lcd_spi_cs);
    let pin_lcd_dc = RefCell::new(pin_lcd_dc);

    // Create shared versions of the pins and SPI bus
    let pin_lcd_spi_cs_1 = SharedPin::new(&pin_lcd_spi_cs);
    let pin_lcd_spi_cs_2 = SharedPin::new(&pin_lcd_spi_cs);
    let mut pin_lcd_spi_cs_3 = SharedPin::new(&pin_lcd_spi_cs);

    let pin_lcd_dc_1 = SharedPin::new(&pin_lcd_dc);
    let pin_lcd_dc_2 = SharedPin::new(&pin_lcd_dc);
    let mut pin_lcd_dc_3 = SharedPin::new(&pin_lcd_dc);

    let lcd_spi_device_1 =
        embedded_hal_bus::spi::RefCellDevice::new(&lcd_spi_bus, pin_lcd_spi_cs_1, NoDelay);
    let lcd_spi_device_2 =
        embedded_hal_bus::spi::RefCellDevice::new(&lcd_spi_bus, pin_lcd_spi_cs_2, NoDelay);

    // Create the display interface s
    let display_interface_1 = SPIDisplayInterface::new(lcd_spi_device_1, pin_lcd_dc_1);
    let mut display_interface_2 = SPIDisplayInterface::new(lcd_spi_device_2, pin_lcd_dc_2);

    let mut display_driver = Gc9a01::new(
        display_interface_1,
        DisplayResolution240x240,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics();

    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Resetting display\r\n");
    display_driver.reset(&mut pin_lcd_reset, &mut timer).ok();
    display_driver.init(&mut timer).ok();

    // Set backlight to 100%
    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Turning on backlight\r\n");
    embedded_hal::digital::v2::OutputPin::set_high(&mut pin_lcd_backlight).unwrap();

    display_driver.set_brightness(Brightness::DIM).ok();

    // display_driver
    //     .set_draw_area((0, 30), (APPLE_FRAME_WIDTH, APPLE_FRAME_HEIGHT))
    //     .ok();

    // display_driver
    //     .set_display_rotation(DisplayRotation::Rotate180)
    //     .ok();

    //--------------------------
    // IMU Initialization
    //--------------------------
    // write_and_poll_usb_once(&mut usb_dev, &mut serial, b"initializing IMU\r\n");

    // Do PIO things for the I2C
    // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Create the I2C driver
    // let mut i2c_pio = i2c_pio::I2C::new(
    //     &mut pio,
    //     pin_imu_i2c_sda,
    //     pin_imu_i2c_scl,
    //     sm0,
    //     100.kHz(),
    //     clocks.system_clock.freq(),
    // );

    // Create the IMU driver
    // let mut qmi8658_device = Qmi8658::new_secondary_address(i2c_pio, timer);

    // TODO: log the device ID
    // match qmi8658_device.get_device_id() {
    //     Ok(_value) => {}
    //     Err(_e) => {}
    // }
    // write_and_poll_usb_once(&mut usb_dev, &mut serial, b"IMU initialized\r\n");

    write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Starting playback loop\r\n");
    //--------------------------
    // Main program loop
    //--------------------------

    // let raw_images = ALL_DATA
    //     .iter()
    //     .map(|data| ImageRaw::<Rgb565, LittleEndian>::new(data, 240))
    //     .collect::<heapless::Vec<_, 14>>();

    // let mut images = raw_images
    //     .iter()
    //     .map(|raw_image| Image::new(raw_image, Point::zero()))
    //     .collect::<heapless::Vec<_, 14>>();

    // let imageraw = ImageRaw::<Rgb565, LittleEndian>::new(DATA_0, 240);
    // let image = Image::new(&imageraw, Point::zero());

    let mut current_image_number = 0;

    let mut said_hello = false;
    // let mut tick: u32 = 0;
    let mut last_screen_update_tick: u64 = timer.get_counter().ticks();
    let mut display_buffer =
        Some(singleton!(: [u8; DISPLAY_BUFFER_SIZE] = [0; DISPLAY_BUFFER_SIZE]).unwrap());

    prepare_apple_frame(
        &mut heatshrink,
        current_image_number,
        display_buffer.as_mut().unwrap(),
    );

    display_driver.clear();
    display_driver.flush().ok();

    let frameskips = [1];
    let mut frameskipindex = 0;
    let mut framewait = frameskips[frameskipindex];

    loop {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            read_serial_and_echo(&mut serial);
        }

        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            // let _ = serial.write(b"Hello, World!\r\n");
            write_serial_full_drop_err(&mut serial, b"Playing...\r\n");

            // let time = timer.get_counter().ticks();
            // let mut text: String<64> = String::new();
            // writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            // let _ = serial.write(b"testing things");
        }

        //--------------------------
        // Update LCD
        //--------------------------
        let now_ticks = timer.get_counter().ticks();
        if last_screen_update_tick + 33200 * framewait < now_ticks {
            frameskipindex = (frameskipindex + 1) % frameskips.len();
            framewait = frameskips[frameskipindex];

            // let mut text: String<64> = String::new();
            // write!(&mut text, "Current timer ticks: {}\r\n", now_ticks).unwrap();
            // write_and_poll_usb_once(&mut usb_dev, &mut serial, text.as_bytes());
            // write_and_poll_usb_once(&mut usb_dev, &mut serial, b"drawing to display\r\n");
            // let before_ticks = timer.get_counter().ticks();
            last_screen_update_tick = now_ticks;
            // display_driver.clear();
            current_image_number = (current_image_number + 1) % apple::APPLE_FRAMES.len();
            // images[current_image_number]
            //     .draw(&mut display_driver)
            //     .unwrap();
            // images[current_image_number]/
            // display_driver
            //     .set_draw_area((0, 0), display_driver.dimensions())
            //     .ok();
            // display_driver.clear_fit().ok();

            // let data_to_send = ALL_DATA[current_image_number];
            // let data_to_send = ALL_DATA[current_image_number];
            // let spi = lcd_spi_bus.borrow_mut();
            // let transfer = dma::single_buffer::Config::new(dma.ch0, data_to_send, spi);

            // display_driver.draw(ALL_DATA[current_image_number]).ok();

            // Set dc high
            // write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Preparing data...\r\n");

            // let apple_frame_start = current_image_number * APPLE_FRAME_BYTES;
            // let apple_frame_end = apple_frame_start + APPLE_FRAME_BYTES;
            // let apple_frame_bytes = &APPLE_DATA[apple_frame_start..apple_frame_end];

            let buf = display_buffer.take().unwrap();

            // Move to display buffer, converting from packed 1-bit to 16-bit color

            // write_and_poll_usb_once(&mut usb_dev, &mut serial, b"Sending data...\r\n");

            Command::MemoryWrite.send(&mut display_interface_2).ok();
            pin_lcd_dc_3.set_high().ok();

            pin_lcd_spi_cs_3.set_low().ok();

            // display_driver.draw(ALL_DATA[current_image_number]).ok();

            let spi_fake = MySpiWrapper::new_empty();
            let spi_wrapper = lcd_spi_bus.replace(spi_fake);
            let spi = spi_wrapper.spi.unwrap();
            let transfer =
                dma::single_buffer::Config::new(dma_spi_channel.take().unwrap(), buf, spi).start();
            let (returned_dma_spi_channel, returned_send_buffer, returned_spi) = transfer.wait();

            pin_lcd_spi_cs_3.set_high().ok();

            // let after_ticks = timer.get_counter().ticks();
            // let mut text: String<64> = String::new();
            // write!(
            //     &mut text,
            //     "Drawing took {} ticks\r\n",
            //     after_ticks - before_ticks
            // )
            // .unwrap();
            // write_and_poll_usb_once(&mut usb_dev, &mut serial, text.as_bytes());

            // Prepare the next frame
            prepare_apple_frame(&mut heatshrink, current_image_number, returned_send_buffer);

            // Return everything so we can use them again
            dma_spi_channel.replace(returned_dma_spi_channel);
            lcd_spi_bus.replace(MySpiWrapper::new(returned_spi));
            display_buffer.replace(returned_send_buffer);

            // draw(&mut display_driver, tick);

            // image.draw(&mut display_driver).unwrap();
            // display_driver.flush().ok();
            // tick += 1;
        }
    }
}

fn prepare_apple_frame(
    heatshrink: &mut HeatshrinkDecoder,
    frame_number: usize,
    display_buffer: &mut [u8; DISPLAY_BUFFER_SIZE],
) {
    let apple_frame_compressed = apple::APPLE_FRAMES[frame_number];
    let mut apple_frame_uncompressed = [0; APPLE_FRAME_BYTES];

    heatshrink.reset();

    // Decompression loop
    let mut total_bytes_sunk = 0;
    let mut total_bytes_polled = 0;
    loop {
        // Sink as much as possible
        loop {
            if total_bytes_sunk >= apple_frame_compressed.len() {
                break;
            }
            let (res, sunk_bytes) = heatshrink.sink(&apple_frame_compressed[total_bytes_sunk..]);
            total_bytes_sunk += sunk_bytes;
            match res {
                heatshrink::HSsinkRes::SinkErrorMisuse => {
                    break;
                }
                heatshrink::HSsinkRes::SinkFull => {
                    break;
                }
                heatshrink::HSsinkRes::SinkOK => {
                    // Continue
                }
            }
            if sunk_bytes == 0 {
                break;
            }
        }
        // Poll as much as possible
        loop {
            let (res, poll_bytes) =
                heatshrink.poll(&mut apple_frame_uncompressed[total_bytes_polled..]);
            total_bytes_polled += poll_bytes;
            match res {
                heatshrink::HSpollRes::PollErrorMisuse => {
                    break;
                }
                heatshrink::HSpollRes::PollEmpty => {
                    break;
                }
                heatshrink::HSpollRes::PollMore => {
                    // Continue
                }
            }
            if poll_bytes == 0 {
                break;
            }
        }
        // If we've sunk all the bytes, finish
        if total_bytes_sunk >= apple_frame_compressed.len() {
            break;
        }
    }

    // Loop finish and poll till there's nothing left
    loop {
        match heatshrink.finish() {
            heatshrink::HSfinishRes::FinishDone => {
                break;
            }
            heatshrink::HSfinishRes::FinishMore => {
                let (res, poll_bytes) =
                    heatshrink.poll(&mut apple_frame_uncompressed[total_bytes_polled..]);
                total_bytes_polled += poll_bytes;
                match res {
                    heatshrink::HSpollRes::PollErrorMisuse => {
                        break;
                    }
                    heatshrink::HSpollRes::PollEmpty => {
                        // Continue
                    }
                    heatshrink::HSpollRes::PollMore => {
                        // Continue
                    }
                }
            }
        }
    }

    // Convert the 1-bit packed data to 16-bit color
    for i in 0..APPLE_FRAME_BITS {
        let byte_index = i / 8;
        let bit_index = 7 - (i % 8);
        let current_bit = (apple_frame_uncompressed[byte_index] >> bit_index) & 1;
        if current_bit != 0 {
            display_buffer[DISPLAY_BUFFER_OFFSET + i * 2] = 0xFF;
            display_buffer[DISPLAY_BUFFER_OFFSET + i * 2 + 1] = 0xFF;
        } else {
            display_buffer[DISPLAY_BUFFER_OFFSET + i * 2] = 0x00;
            display_buffer[DISPLAY_BUFFER_OFFSET + i * 2 + 1] = 0x00;
        }
    }
}

// 17600 is the time to beat

fn write_serial_full_drop_err<B: usb_device::bus::UsbBus>(serial: &mut SerialPort<B>, buf: &[u8]) {
    let mut wr_ptr = buf;
    while !wr_ptr.is_empty() {
        match serial.write(wr_ptr) {
            Ok(len) => wr_ptr = &wr_ptr[len..],
            // On error, just drop unwritten data.
            // One possible error is Err(WouldBlock), meaning the USB
            // write buffer is full.
            Err(_) => break,
        };
    }
}

fn write_and_poll_usb_once<B: usb_device::bus::UsbBus>(
    usb_dev: &mut UsbDevice<'_, B>,
    serial: &mut SerialPort<'_, B>,
    buf: &[u8],
) {
    write_serial_full_drop_err(serial, buf);
    if usb_dev.poll(&mut [serial]) {
        read_serial_and_echo(serial);
    }
}

fn read_serial_and_echo<B: usb_device::bus::UsbBus>(serial: &mut SerialPort<'_, B>) {
    let mut buf = [0u8; 64];
    match serial.read(&mut buf) {
        Err(_e) => {
            // Do nothing
        }
        Ok(0) => {
            // Do nothing
        }
        Ok(count) => {
            // Convert to upper case
            buf.iter_mut().take(count).for_each(|b| {
                b.make_ascii_uppercase();
            });
            // Send back to the host
            let mut wr_ptr = &buf[..count];
            while !wr_ptr.is_empty() {
                match serial.write(wr_ptr) {
                    Ok(len) => wr_ptr = &wr_ptr[len..],
                    // On error, just drop unwritten data.
                    // One possible error is Err(WouldBlock), meaning the USB
                    // write buffer is full.
                    Err(_) => break,
                };
            }
        }
    }
}

pub struct SharedPin<'a, Pin> {
    pin: &'a RefCell<Pin>,
}

impl<'a, Pin> SharedPin<'a, Pin> {
    /// Create a new [`RefCellDevice`].
    #[inline]
    pub fn new(pin: &'a RefCell<Pin>) -> Self {
        Self { pin }
    }
}

// impl<'a, Pin> embedded_hal::digital::OutputPin for SharedPin<'a, Pin>
// where
//     Pin: embedded_hal::digital::OutputPin,
// {
//     /// Borrows the RefCell and calls set_low() on the pin
//     fn set_low(&mut self) {
//         self.pin.borrow_mut().set_low();
//     }

//     /// Borrows the RefCell and calls set_high() on the pin
//     fn set_high(&mut self) {
//         self.pin.borrow_mut().set_high()
//     }
// }

impl<'a, Pin> embedded_hal_new::digital::ErrorType for SharedPin<'a, Pin> {
    type Error = Infallible;
}

impl<'a, Pin> OutputPinNew for SharedPin<'a, Pin>
where
    Pin: embedded_hal::digital::v2::OutputPin<Error = Infallible>,
{
    /// Borrows the RefCell and calls set_low() on the pin
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.borrow_mut().set_low()
    }

    /// Borrows the RefCell and calls set_high() on the pin
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.borrow_mut().set_high()
    }
}
