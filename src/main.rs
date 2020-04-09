#![no_std]
#![no_main]

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use feather_m0 as hal;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::fmt::Write as cw;
use embedded_hal::serial::Write;
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

static mut LIGHT: Option<hal::gpio::Pa17<hal::gpio::Output<hal::gpio::OpenDrain>>> = None;

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static USB_BUS: Mutex<RefCell<Option<UsbDevice<UsbBus>>>> = Mutex::new(RefCell::new(None));
static USB_SERIAL: Mutex<RefCell<Option<SerialPort<UsbBus>>>> = Mutex::new(RefCell::new(None));


#[panic_handler]
fn panic_impl(_info: &core::panic::PanicInfo) -> ! {
    match unsafe { LIGHT.as_mut() } {
        None => loop { },
        Some(led) => {
            // (0..100_000).sum::<u32>();
            loop {
                led.set_high().unwrap_or(());
                (0..10_000).sum::<u32>();
                led.set_low().unwrap_or(());
                (0..10_000).sum::<u32>();
                led.set_high().unwrap_or(());
                (0..10_000).sum::<u32>();
                led.set_low().unwrap_or(());
                (0..60_000).sum::<u32>();
            }
        },
    }
}


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
    // let mut red_led = pins.d13.into_open_drain_output(&mut pins.port);

    // let mut test = Some(5);
    // let mut u = test.as_mut().unwrap();

    let red_led = unsafe {
        LIGHT = Some(pins.d13.into_open_drain_output(&mut pins.port));
        LIGHT.as_mut().unwrap()
    };

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

    red_led.set_high().unwrap();

    cortex_m::interrupt::free(|cs| {
        let serial = Some(SerialPort::new(&bus_allocator));
        let cell = USB_SERIAL.borrow(cs);
        // red_led.set_low().unwrap();
        // delay.delay_ms(1u8);
        cell.replace(serial);
    });

    cortex_m::interrupt::free(|cs| {
        let bus = Some(UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build());
        USB_BUS.borrow(cs).replace(bus);
    });

    // unsafe {
    //     USB_BUS = Some(UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
    //         .manufacturer("Fake company")
    //         .product("Serial port")
    //         .serial_number("TEST")
    //         .device_class(USB_CLASS_CDC)
    //         .build());
    //     // USB_BUS.as_ref().unwrap()
    // };

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }


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

        test_write();
    }
}


fn test_write() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut bus) = USB_BUS.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut serial) = USB_SERIAL.borrow(cs).borrow_mut().deref_mut() {
                bus.poll(&mut [serial]);
                if serial.dtr() {
                    let stream = serial as &mut dyn Write<u8, Error=UsbError>;
                    writeln!(stream, "Hello World").unwrap();
                }
            }
        }
    });
}


#[interrupt]
fn USB() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut bus) = USB_BUS.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut serial) = USB_SERIAL.borrow(cs).borrow_mut().deref_mut() {
                bus.poll(&mut [serial]);

                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    let stream = serial as &mut dyn Write<u8, Error=UsbError>;
                    for (i, &c) in buf[..count].iter().enumerate() {
                        // only 32 to 126 are printable
                        let printable = if c >= 32 && c <= 126 {c as char} else {' '};
                        writeln!(stream, "[0x{:x?} ( {} ), {}/{}]", c, printable, i+1, count).unwrap();
                    }
                    writeln!(stream, "").unwrap();
                }
            }
        }
    });
}