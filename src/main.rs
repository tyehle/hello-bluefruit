#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

extern crate feather_m0 as hal;

use cortex_m_rt::entry;
use cortex_m::peripheral::NVIC;
use atsamd_hal::common::gpio::IntoFunction;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::{UsbVidPid, UsbDevice, UsbDeviceBuilder};
use usbd_serial::{SerialPort, USB_CLASS_CDC};


static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;


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
    let mut red_led = pins.d13.into_open_drain_output(&mut pins.port);
    let mut delay = Delay::new(core.SYST, &mut clocks);


    let bus_allocator = unsafe {
        let gclk0 = clocks.gclk0();
        let usb_clock = &clocks.usb(&gclk0).unwrap();
        USB_ALLOCATOR = Some(UsbBusAllocator::new(UsbBus::new(
            usb_clock,
            &mut peripherals.PM,
            pins.usb_dm.into_function(&mut pins.port),
            pins.usb_dp.into_function(&mut pins.port),
            peripherals.USB
        )));

        // USB_ALLOCATOR = Some(hal::usb_allocator(
        //     peripherals.USB,
        //     &mut clocks,
        //     &mut peripherals.PM,
        //     pins.usb_dm,
        //     pins.usb_dp,
        //     &mut pins.port,
        // ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

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
    }
}

#[interrupt]
fn USB() {

}