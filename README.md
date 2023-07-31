# psx-usb

USB adapter for Playstation DualShock controllers.
Requires a [multitap](https://en.wikipedia.org/wiki/PlayStation_Multitap) adapter.

WARNING: Not actually tested with physical playstation controllers.

## As USB interface for [BlueRetro](https://github.com/darthcloud/BlueRetro)

Instead of connecting actual playstation controllers you can use an ESP32 board
running BlueRetro. BlueRetro receives data from wireless bluetooth controllers
like Switch, PS3, PS4, PS5, Xbox One, Xbox Series X|S.

``` text
                        BT
Bluetooth Controller <------> BlueRetro <-\
                                          |
            USB: XInput                   |PSX SPI protocol
Computer <---------------> psx-usb <------/
```

The PSX controller [communication protocol](https://hackaday.io/project/170365-blueretro/log/186471-playstation-playstation-2-spi-interface)
is an open drain SPI (LSB first) with additional acknowledge line. The RP2040s PIO makes it easy implement this custom SPI protocol.

A total of 5 wires (plus VCC and GND) is required to connect the two boards:

| Pin Description | BlueRetro  | psx-usb      | Comment         |
| --------------- | ---------- | ------------ | --------------- |
|                 | ESP32 Pin  | RPI Pico Pin |                 |
| SCK / CLK       | 33         | 2            |                 |
| TXD / MOSI      | 32         | 3            |                 |
| RXD / MISO      | 19         | 4            | Pullup required |
| DTR / CS        | 34         | 5            |                 |
| DSR / ACK       | 21         | 6            | Pullup required |

## USB Protocol

Its easy to create a HID composite device which exposes many controllers on a
single USB connection. Unfortunately the button mapping is not well standardized
and usually requires additional setup/remapping steps on the computer.

The alternative is to emulate Xbox 360 controllers (aka XInput devices) which
have a well defined button arrangement understood by most games.
Unfortunately the windows driver only accepts exactly one wired Xbox 360
controller per USB device.

To overcome this limitation I had to reverse engineer the USB protocol of a
"Xbox 360 Wireless Receiver for Windows" which supports up to 4 wireless
controllers.

Resources I found useful during the process:

* <https://www.partsnotincluded.com/understanding-the-xbox-360-wired-controllers-usb-data/>
* <https://www.partsnotincluded.com/xbox-360-controller-led-animations-info/>
* <https://github.com/torvalds/linux/blob/master/drivers/input/joystick/xpad.c>
* <https://computerquip.wordpress.com/2013/12/16/i-hate-microsoft-but-for-the-better-good/>
* <https://github.com/360Controller/360Controller>

## License

Licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
