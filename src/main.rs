#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

extern crate feather_m0 as hal;

use core::cell::RefCell;

use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use hal::gpio::IntoFunction;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::{UsbVidPid, UsbDevice, UsbDeviceBuilder, UsbError};
use usbd_serial::{SerialPort, USB_CLASS_CDC};


static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static USB_BUS: Mutex<RefCell<Option<UsbDevice<UsbBus>>>> = Mutex::new(RefCell::new(None));
static USB_SERIAL: Mutex<RefCell<Option<SerialPort<UsbBus>>>> = Mutex::new(RefCell::new(None));


// fn configure_usb() {

// }


// fn configure_led(pins: &mut hal::Pins) -> impl embedded_hal::digital::v2::OutputPin {
//     pins.d13.into_open_drain_output(&mut pins.port)
// }

// fn configure_led(pins: &mut hal::Pins) -> hal::gpio::Pa17<hal::gpio::Output<hal::gpio::OpenDrain>> {
//     pins.d13.into_open_drain_output(&mut pins.port)
// }


#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut pins = hal::Pins::new(peripherals.PORT);

    // let mut red_led = configure_led(&mut pins);
    let mut red_led = pins.d13.into_open_drain_output(&mut pins.port);

    let mut delay = Delay::new(core.SYST, &mut clocks);

    let gclk0 = clocks.gclk0();
    let usb_clock = &clocks.usb(&gclk0).unwrap();
    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(UsbBusAllocator::new(UsbBus::new(
            usb_clock,
            &mut peripherals.PM,
            pins.usb_dm.into_function(&mut pins.port),
            pins.usb_dp.into_function(&mut pins.port),
            peripherals.USB
        )));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    cortex_m::interrupt::free(|cs| {
        // USB_ALLOCATOR.borrow(cs).replace(Some(bus_allocator));

        let usb_serial = SerialPort::new(&bus_allocator);
        USB_SERIAL.borrow(cs).replace(Some(usb_serial));

        let usb_bus = UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();
        USB_BUS.borrow(cs).replace(Some(usb_bus));
    });

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }


    // write_usb("Starting up!\n".as_bytes());
    // write_usb("------------\n".as_bytes());


    let mut delay_time = 200i16;
    let mut delay_change = 100i16;
    loop {
        delay.delay_ms(delay_time as u16);
        red_led.set_high().unwrap();
        delay.delay_ms(delay_time as u16);
        red_led.set_low().unwrap();
        if delay_time <= 100 || delay_time > 1000 {
            delay_change *= -1;
        }
        delay_time += delay_change;

        // let mut buf = [0; 16];
        // read_usb(&mut buf);
        // write_usb("Hello :)\n".as_bytes());
    }
}


// fn read_usb(buffer: &mut [u8]) -> usize {
//     unsafe {
//         match USB_SERIAL.as_mut() {
//             None => 0,
//             Some(serial) => serial.read(buffer).unwrap_or(0),
//         }
//     }
// }


// fn write_usb(data: &[u8]) -> usize {
//     unsafe {
//         match USB_SERIAL.as_mut() {
//             None => 0,
//             Some(serial) => serial.write(data).unwrap_or(0),
//         }
//     }
// }


#[interrupt]
fn USB() {
    cortex_m::interrupt::free(|cs| {
        use core::ops::DerefMut;
        use core::fmt::Write as cw;
        use embedded_hal::serial::Write;

        if let Some(ref mut bus) = USB_BUS.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut serial) = USB_SERIAL.borrow(cs).borrow_mut().deref_mut() {
                bus.poll(&mut [serial]);

                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    let stream = serial as &mut dyn Write<u8, Error=UsbError>;
                    for (i, c) in buf[..count].iter().enumerate() {
                        writeln!(stream, "[{}, {}, {}/{}]", *c as char, c, i+1, count).unwrap();
                    }
                    writeln!(stream, "-----").unwrap();
                }
            }
        }
    });
}