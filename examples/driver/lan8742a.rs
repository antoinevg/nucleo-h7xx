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

use jacktrip::Address;
use jacktrip::driver::SocketDriver;
use jacktrip::{Error, Result};


// - driver::lan8742a::Socket -------------------------------------------------

pub struct Socket <'a> {
    socket_handle: SocketHandle,
    remote: IpEndpoint,
    _marker: core::marker::PhantomData<&'a ()>
}


//impl SocketDriver for Socket {
impl<'a> SocketDriver for Socket<'a> {
    fn bind(address: &str, port: u16) -> Result<Socket<'a>> {
        //hprintln!("driver::lan8742a::Socket::bind -> {}:{}", address, port).unwrap();

        let ip_address = parse_ip_address(address, [0_u8; 4])?;
        let endpoint = IpEndpoint::new(Ipv4Address::from_bytes(&ip_address).into(), port);

        let ethernet_interface = unsafe { nucleo::ethernet::ETHERNET_INTERFACE.as_mut().unwrap() };
        let udp_socket_handle = ethernet_interface.new_udp_socket();

        let ethernet_interface = unsafe { nucleo::ethernet::ETHERNET_INTERFACE.as_mut().unwrap() };
        let mut udp_socket = ethernet_interface.sockets.as_mut().unwrap().get::<UdpSocket>(udp_socket_handle);

        match udp_socket.bind(endpoint) {
            Ok(()) => (),
            Err(e) => {
                hprintln!("Failed to bind socket to endpoint: {:?}", endpoint).unwrap();
                return Err(Error::ToDo);
            }
        }

        Ok(Socket {
            socket_handle: udp_socket_handle,
            remote: IpEndpoint::UNSPECIFIED,
            _marker: core::marker::PhantomData,
        })
    }

    fn connect(&mut self, address: &str, port: u16) -> Result<()> {
        //hprintln!("driver::lan8742a::Socket::connect -> {}:{}", address, port).unwrap();

        let ip_address = parse_ip_address(address, [0_u8; 4])?;
        self.remote = IpEndpoint::new(Ipv4Address::from_bytes(&ip_address).into(), port);

        Ok(())
    }

    fn send(&mut self, buffer: &[u8]) -> Result<usize> {
        let ethernet_interface = unsafe { nucleo::ethernet::ETHERNET_INTERFACE.as_mut().unwrap() };
        let mut udp_socket = ethernet_interface.sockets.as_mut().unwrap().get::<UdpSocket>(self.socket_handle);

        let gpiob = unsafe { &mut pac::Peripherals::steal().GPIOB };
        let gpioe = unsafe { &mut pac::Peripherals::steal().GPIOE };

        let mut receive_buffer = [0_u8; jacktrip::packet::PACKET_SIZE];
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

        /*let mut count = 0;
        while !udp_socket.can_send() && count < 5_000 {
            gpiob.bsrr.write(|w| w.bs14().set_bit());
            if udp_socket.can_recv() {
                match udp_socket.recv_slice(&mut receive_buffer) {
                    Ok(_bytes_received) => (),
                    Err(e) => hprintln!("driver::lan8742a::Socket::recv error: {:?}", e).unwrap(),
                }
            }
            count += 1;
        }
        gpiob.bsrr.write(|w| w.br14().set_bit());*/

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
    }

    fn receive(&mut self, buffer: &mut [u8]) -> Result<(usize, Address)> {
        Ok((0, Address { host: "0.0.0.0", port: 0 }))
    }
}


// - helpers ------------------------------------------------------------------

fn parse_ip_address(address: &str, mut octets: [u8; 4]) -> Result<[u8; 4]> {
    for (n, octet) in address.split('.').enumerate() {
        octets[n] = match str::parse(octet) {
            Ok(b) => b,
            Err(_) => {
                hprintln!("Invalid ip address: {} -> '{}'", address, octet).unwrap();
                return Err(Error::ToDo);
            }
        };
    }
    Ok(octets)
}
