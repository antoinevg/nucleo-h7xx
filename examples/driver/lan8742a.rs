use cortex_m_semihosting::hprintln;

use stm32h7xx_hal as hal;
use hal::hal::digital::v2::OutputPin;
use hal::gpio::Speed::*;
use hal::{ethernet, ethernet::PHY};
use hal::{pac, prelude::*};

use nucleo_h745zi as nucleo;

use smoltcp;
use smoltcp::iface::{
    EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache,
    Route, Routes,
};
use smoltcp::socket::{SocketHandle, SocketSet, SocketSetItem};
use smoltcp::socket::{UdpSocket, UdpSocketBuffer, UdpPacketMetadata};
use smoltcp::storage::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv6Cidr, IpEndpoint, Ipv4Address};

use jacktrip::{Error, Result};
use jacktrip::net::{Endpoint, Octets, Traits};
use jacktrip::driver::SocketDriver;


// - driver::lan8742a::Socket -------------------------------------------------

pub struct Socket <'a> {
    socket_handle: SocketHandle,
    remote: IpEndpoint,
    _marker: core::marker::PhantomData<&'a ()>
}

impl<'a> SocketDriver for Socket<'a> {
    fn bind(local: &Endpoint) -> Result<Socket<'a>> {
        //hprintln!("driver::lan8742a::Socket::bind -> {:?}", local).unwrap();

        let local = IpEndpoint::new(
            Ipv4Address::from_bytes(&local.octets).into(),
            local.port
        );

        let result = nucleo::ethernet::Interface::interrupt_free(|ethernet_interface| {
            let udp_socket_handle = ethernet_interface.new_udp_socket();
            let mut udp_socket =
                ethernet_interface.sockets.as_mut().unwrap().get::<UdpSocket>(udp_socket_handle);

            match udp_socket.bind(local) {
                Ok(()) => return Ok(udp_socket_handle),
                Err(e) => {
                    hprintln!("Failed to bind socket to endpoint: {:?}", local).unwrap();
                    return Err(Error::ToDo);
                }
            }
        });

        let udp_socket_handle = match result {
            Ok(udp_socket_handle) => udp_socket_handle,
            Err(e) => {
                hprintln!("Failed to bind socket to endpoint: {:?}", local).unwrap();
                return Err(Error::ToDo);
            }
        };

        Ok(Socket {
            socket_handle: udp_socket_handle,
            remote: IpEndpoint::UNSPECIFIED,
            _marker: core::marker::PhantomData,
        })
    }

    fn connect(&mut self, remote: &Endpoint) -> Result<()> {
        //hprintln!("driver::lan8742a::Socket::connect -> {:?}", remote).unwrap();
        self.remote = IpEndpoint::new(
            Ipv4Address::from_bytes(&remote.octets).into(),
            remote.port
        );

        Ok(())
    }

    fn send(&mut self, buffer: &[u8]) -> Result<usize> {
        let mut receive_buffer = [0_u8; jacktrip::packet::PACKET_SIZE];
        let gpiob = unsafe { &mut pac::Peripherals::steal().GPIOB };
        let gpioe = unsafe { &mut pac::Peripherals::steal().GPIOE };

        let result = nucleo::ethernet::Interface::interrupt_free(|ethernet_interface| {
            let mut udp_socket =
                ethernet_interface.sockets.as_mut().unwrap().get::<UdpSocket>(self.socket_handle);


            if udp_socket.can_recv() {
                match udp_socket.recv_slice(&mut receive_buffer) {
                    Ok(_bytes_received) => {
                        gpioe.bsrr.write(|w| w.br1().set_bit());
                    },
                    Err(e) => {
                        //hprintln!("driver::lan8742a::Socket::recv error: {:?}", e).unwrap();
                        gpioe.bsrr.write(|w| w.bs1().set_bit());
                    },
                }
            }

            if udp_socket.can_send() {
                match udp_socket.send_slice(buffer, self.remote) {
                    Ok(()) => (),
                    Err(smoltcp::Error::Exhausted) => (), // TODO figure out if this is a problem
                    Err(e) => hprintln!("driver::lan8742a::Socket::send error: {:?}", e).unwrap(),
                }
                gpiob.bsrr.write(|w| w.br14().set_bit());
            } else {
                //hprintln!("driver::lan8742a::Socket::send error: can't send").unwrap();
                gpiob.bsrr.write(|w| w.bs14().set_bit());
            }

            Ok(buffer.len())
        });

        result
    }

    fn receive(&mut self, buffer: &mut [u8]) -> Result<(usize, Endpoint)> {
        let gpioe = unsafe { &mut pac::Peripherals::steal().GPIOE };

        let result = nucleo::ethernet::Interface::interrupt_free(|ethernet_interface| {
            let mut udp_socket =
                ethernet_interface.sockets.as_mut().unwrap().get::<UdpSocket>(self.socket_handle);
            if !udp_socket.can_recv() { // TODO should we block if there are no packets available in buffer?
                return Err(smoltcp::Error::Exhausted);
            }
            udp_socket.recv_slice(buffer)
        });

        match result {
            Ok((bytes_received, endpoint)) => {
                gpioe.bsrr.write(|w| w.br1().set_bit());
                let mut octets = Octets::new();
                octets.clone_from_slice(&endpoint.addr.as_bytes()[0..3]);
                Ok((bytes_received, Endpoint {
                    octets: octets,
                    port: endpoint.port
                }))
            },
            Err(e) => {
                hprintln!("driver::lan8742a::Socket::recv error: {:?}", e).unwrap();
                gpioe.bsrr.write(|w| w.bs1().set_bit());
                Err(Error::Receive)
            }
        }
    }
}
