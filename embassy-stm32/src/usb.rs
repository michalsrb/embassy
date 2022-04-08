#![macro_use]

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};
use core::task::Poll;
use embassy::interrupt::InterruptExt;
use embassy::time::{Duration, Timer};
use embassy::util::Unborrow;
use embassy::waitqueue::AtomicWaker;
use embassy_hal_common::unborrow;
use embassy_usb::control::Request;
use embassy_usb::driver::{self, EndpointError, Event};
use embassy_usb::types::{EndpointAddress, EndpointInfo, EndpointType, UsbDirection};
use futures::future::poll_fn;
use futures::Future;

use crate::gpio::low_level::AFType;
use crate::interrupt::Interrupt;
use crate::rcc::low_level::RccPeripheral;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP0_WAKER: AtomicWaker = NEW_AW;
static EP_IN_WAKERS: [AtomicWaker; 8] = [NEW_AW; 8];
static EP_OUT_WAKERS: [AtomicWaker; 8] = [NEW_AW; 8];
static READY_ENDPOINTS: AtomicU32 = AtomicU32::new(0);

pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    alloc_in: Allocator,
    alloc_out: Allocator,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new(
        _usb: impl Unborrow<Target = T> + 'd,
        irq: impl Unborrow<Target = T::Interrupt> + 'd,
        dp: impl Unborrow<Target = impl DpPin<T>> + 'd,
        dm: impl Unborrow<Target = impl DmPin<T>> + 'd,
    ) -> Self {
        unborrow!(irq, dp, dm);
        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        unsafe {
            dp.set_as_af(dp.af_num(), AFType::OutputPushPull);
            dm.set_as_af(dm.af_num(), AFType::OutputPushPull);
        }

        Self {
            phantom: PhantomData,
            alloc_in: Allocator::new(),
            alloc_out: Allocator::new(),
        }
    }

    fn on_interrupt(_: *mut ()) {
        unsafe {
            let regs = T::regs();
            let x = regs.istr().read().0;
            info!("USB IRQ: {:08x}", x);
        }
    }

    fn set_stalled(ep_addr: EndpointAddress, stalled: bool) {
        let regs = T::regs();
        // TODO
    }

    fn is_stalled(ep_addr: EndpointAddress) -> bool {
        let regs = T::regs();
        // TODO
        false
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;
    type EnableFuture = impl Future<Output = Self::Bus> + 'd;

    fn alloc_endpoint_in(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        let index = self
            .alloc_in
            .allocate(ep_addr, ep_type, max_packet_size, interval)?;
        let ep_addr = EndpointAddress::from_parts(index, UsbDirection::In);
        Ok(Endpoint::new(EndpointInfo {
            addr: ep_addr,
            ep_type,
            max_packet_size,
            interval,
        }))
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
        let index = self
            .alloc_out
            .allocate(ep_addr, ep_type, max_packet_size, interval)?;
        let ep_addr = EndpointAddress::from_parts(index, UsbDirection::Out);
        Ok(Endpoint::new(EndpointInfo {
            addr: ep_addr,
            ep_type,
            max_packet_size,
            interval,
        }))
    }

    fn alloc_control_pipe(
        &mut self,
        max_packet_size: u16,
    ) -> Result<Self::ControlPipe, driver::EndpointAllocError> {
        self.alloc_endpoint_out(Some(0x00.into()), EndpointType::Control, max_packet_size, 0)?;
        self.alloc_endpoint_in(Some(0x80.into()), EndpointType::Control, max_packet_size, 0)?;
        Ok(ControlPipe {
            _phantom: PhantomData,
            max_packet_size,
        })
    }

    fn enable(self) -> Self::EnableFuture {
        async move {
            let regs = T::regs();

            unsafe {
                <T as RccPeripheral>::enable();
                <T as RccPeripheral>::reset();

                regs.cntr().write(|w| w.set_pdwn(true));

                Timer::after(Duration::from_millis(100)).await;

                regs.btable().write(|w| w.set_btable(0));
                regs.cntr().write(|w| {
                    w.set_pdwn(false);
                    w.set_fres(false);
                    w.set_resetm(true);
                    w.set_suspm(true);
                    w.set_wkupm(true);
                    w.set_ctrm(true);
                });

                #[cfg(usb_v2)]
                regs.bcdr().write(|w| w.set_dppu(true))
            }

            trace!("enabled");

            Bus {
                phantom: PhantomData,
                alloc_in: self.alloc_in,
                alloc_out: self.alloc_out,
            }
        }
    }
}

pub struct Bus<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    alloc_in: Allocator,
    alloc_out: Allocator,
}

impl<'d, T: Instance> driver::Bus for Bus<'d, T> {
    type PollFuture<'a> = impl Future<Output = Event> + 'a where Self: 'a;

    fn poll<'a>(&'a mut self) -> Self::PollFuture<'a> {
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());
            let regs = T::regs();
            // TODO

            Poll::Pending
        })
    }

    #[inline]
    fn reset(&mut self) {
        self.set_configured(false);
    }

    #[inline]
    fn set_configured(&mut self, configured: bool) {
        let regs = T::regs();

        // TODO

        for i in 1..=7 {
            In::waker(i).wake();
            Out::waker(i).wake();
        }
    }

    #[inline]
    fn set_device_address(&mut self, _addr: u8) {
        // Nothing to do, the peripheral handles this.
    }

    fn set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        Driver::<T>::set_stalled(ep_addr, stalled)
    }

    fn is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        Driver::<T>::is_stalled(ep_addr)
    }

    #[inline]
    fn suspend(&mut self) {
        let regs = T::regs();
        // TODO
    }

    #[inline]
    fn resume(&mut self) {
        let regs = T::regs();
        // TODO
    }
}

pub enum Out {}
pub enum In {}

trait EndpointDir {
    fn waker(i: usize) -> &'static AtomicWaker;
}

impl EndpointDir for In {
    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &EP_IN_WAKERS[i - 1]
    }
}

impl EndpointDir for Out {
    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &EP_OUT_WAKERS[i - 1]
    }
}

pub struct Endpoint<'d, T: Instance, Dir> {
    _phantom: PhantomData<(&'d mut T, Dir)>,
    info: EndpointInfo,
}

impl<'d, T: Instance, Dir> Endpoint<'d, T, Dir> {
    fn new(info: EndpointInfo) -> Self {
        Self {
            info,
            _phantom: PhantomData,
        }
    }
}

impl<'d, T: Instance, Dir: EndpointDir> driver::Endpoint for Endpoint<'d, T, Dir> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    fn set_stalled(&self, stalled: bool) {
        Driver::<T>::set_stalled(self.info.addr, stalled)
    }

    fn is_stalled(&self) -> bool {
        Driver::<T>::is_stalled(self.info.addr)
    }

    type WaitEnabledFuture<'a> = impl Future<Output = ()> + 'a where Self: 'a;

    fn wait_enabled(&mut self) -> Self::WaitEnabledFuture<'_> {
        let i = self.info.addr.index();
        assert!(i != 0);

        poll_fn(move |cx| {
            Dir::waker(i).register(cx.waker());
            Poll::Pending

            /*
            // TODO
            if Dir::is_enabled(T::regs(), i) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
                 */
        })
    }
}

impl<'d, T: Instance> driver::EndpointOut for Endpoint<'d, T, Out> {
    type ReadFuture<'a> = impl Future<Output = Result<usize, EndpointError>> + 'a where Self: 'a;

    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            todo!();
        }
    }
}

impl<'d, T: Instance> driver::EndpointIn for Endpoint<'d, T, In> {
    type WriteFuture<'a> = impl Future<Output = Result<(), EndpointError>> + 'a where Self: 'a;

    fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
        async move {
            todo!();
        }
    }
}

pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

impl<'d, T: Instance> driver::ControlPipe for ControlPipe<'d, T> {
    type SetupFuture<'a> = impl Future<Output = Request> + 'a where Self: 'a;
    type DataOutFuture<'a> = impl Future<Output = Result<usize, EndpointError>> + 'a where Self: 'a;
    type DataInFuture<'a> = impl Future<Output = Result<(), EndpointError>> + 'a where Self: 'a;

    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    fn setup<'a>(&'a mut self) -> Self::SetupFuture<'a> {
        async move {
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
            // TODO

            let buf = [0; 8];
            let req = Request::parse(&buf);
            req
        }
    }

    fn data_out<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::DataOutFuture<'a> {
        async move {
            let regs = T::regs();
            todo!();
        }
    }

    fn data_in<'a>(&'a mut self, buf: &'a [u8], last_packet: bool) -> Self::DataInFuture<'a> {
        async move {
            todo!();
        }
    }

    fn accept(&mut self) {
        let regs = T::regs();
        todo!();
    }

    fn reject(&mut self) {
        let regs = T::regs();
        todo!();
    }
}

fn dma_start() {
    compiler_fence(Ordering::Release);
}

fn dma_end() {
    compiler_fence(Ordering::Acquire);
}

struct Allocator {
    used: u16,
    // Buffers can be up to 64 Bytes since this is a Full-Speed implementation.
    lens: [u8; 9],
}

impl Allocator {
    fn new() -> Self {
        Self {
            used: 0,
            lens: [0; 9],
        }
    }

    fn allocate(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> Result<usize, driver::EndpointAllocError> {
        // Endpoint addresses are fixed in hardware:
        // - 0x80 / 0x00 - Control        EP0
        // - 0x81 / 0x01 - Bulk/Interrupt EP1
        // - 0x82 / 0x02 - Bulk/Interrupt EP2
        // - 0x83 / 0x03 - Bulk/Interrupt EP3
        // - 0x84 / 0x04 - Bulk/Interrupt EP4
        // - 0x85 / 0x05 - Bulk/Interrupt EP5
        // - 0x86 / 0x06 - Bulk/Interrupt EP6
        // - 0x87 / 0x07 - Bulk/Interrupt EP7
        // - 0x88 / 0x08 - Isochronous

        // Endpoint directions are allocated individually.

        let alloc_index = if let Some(ep_addr) = ep_addr {
            match (ep_addr.index(), ep_type) {
                (0, EndpointType::Control) => {}
                (8, EndpointType::Isochronous) => {}
                (n, EndpointType::Bulk) | (n, EndpointType::Interrupt) if n >= 1 && n <= 7 => {}
                _ => return Err(driver::EndpointAllocError),
            }

            ep_addr.index()
        } else {
            match ep_type {
                EndpointType::Isochronous => 8,
                EndpointType::Control => 0,
                EndpointType::Interrupt | EndpointType::Bulk => {
                    // Find rightmost zero bit in 1..=7
                    let ones = (self.used >> 1).trailing_ones() as usize;
                    if ones >= 7 {
                        return Err(driver::EndpointAllocError);
                    }
                    ones + 1
                }
            }
        };

        if self.used & (1 << alloc_index) != 0 {
            return Err(driver::EndpointAllocError);
        }

        self.used |= 1 << alloc_index;
        self.lens[alloc_index] = max_packet_size as u8;

        Ok(alloc_index)
    }
}

pub(crate) mod sealed {
    pub trait Instance {
        fn regs() -> crate::pac::usb::Usb;
    }
}

pub trait Instance: sealed::Instance + RccPeripheral + 'static {
    type Interrupt: Interrupt;
}

// Internal PHY pins
pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

foreach_interrupt!(
    ($inst:ident, usb, $block:ident, HP, $irq:ident) => {
        impl sealed::Instance for crate::peripherals::$inst {
            fn regs() -> crate::pac::usb::Usb {
                crate::pac::$inst
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::$irq;
        }
    };

);

#[cfg(any(stm32l0, stm32l1))]
const DP_PULL_UP_FEATURE: bool = true;
#[cfg(any(stm32f1, stm32f3, stm32l4, stm32l5))]
const DP_PULL_UP_FEATURE: bool = false;

const EP_MEMORY: *const () = 0x4000_6000 as _;

#[cfg(any(stm32f1, stm32l1, stm32f303xb, stm32f303xc))]
const EP_MEMORY_SIZE: usize = 512;
#[cfg(any(stm32l0, stm32l4, stm32l5, stm32f303xd, stm32f303xe))]
const EP_MEMORY_SIZE: usize = 1024;

#[cfg(any(stm32f1, stm32l1, stm32f303xb, stm32f303xc))]
const EP_MEMORY_ACCESS_2X16: bool = false;
#[cfg(any(stm32l0, stm32l4, stm32l5, stm32f303xd, stm32f303xe))]
const EP_MEMORY_ACCESS_2X16: bool = true;
