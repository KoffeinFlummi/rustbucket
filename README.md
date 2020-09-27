rustbucket
==========

Access your car's diagnostics via the OBD2 connector with a BeagleBone Blue and Rust (and maybe some MOSFETs). This was inspired by [Marcelo Sacchetin's post about using the BeagleBone Blue's integrated CAN bus transceiver](https://medium.com/bugbountywriteup/car-hacking-with-python-part-1-data-exfiltration-gps-and-obdii-can-bus-69bc6b101fd1). If you want to read the diagnostics for older vehicles, things are a bit more complicated than that. For older VWs for instance, you will need to talk to the vehicle's K line using the KWP1281 protocol. This requires additional hardware to convert the BeagleBone's 3.3V logic level to 12V.

This was developed on/for the BeagleBone Blue, but the K & L line protocols should be just as usable on a BeagleBone Black. As far as I know, accessing the CAN bus on the Black requires the addition of a CAN transceiver.

<p align="center">
    <a href="https://github.com/KoffeinFlummi/rustbucket/blob/master/doc/golf4.jpg?raw=true">
        <img src="https://github.com/KoffeinFlummi/rustbucket/blob/master/doc/golf4.jpg?raw=true" />
    </a>
</p>

# Disclaimer

Hooking stuff up to your vehicle's OBD2 connector can lead to anything from blown fuses to broken hardware. There are [reports online of attempting to communicate with certain VW airbag controllers via KWP1281 bricking them](https://www.ross-tech.com/vag-com/vw_issues.html). There are also apparently [instances of clearing the fault codes using KWP1281 leading to deployed airbags for certain errors](http://wiki.ross-tech.com/wiki/index.php/01217).

I have tested the software and hardware described here successfully on some vehicles/ECUs (see below). However, **I cannot provide any guarantee that using this will not cause permanent damage to you or your vehicle. Proceed at your own risk. Also, don't fuck with airbags please.**

# Protocols

<p align="center">
    <a href="https://canlogger1000.csselectronics.com/img/OBD2-Connector-DLC-Data-Link-16-Pin-Out-J1962-Explained-What-Is_v2.png">
        <img width=400 src="https://canlogger1000.csselectronics.com/img/OBD2-Connector-DLC-Data-Link-16-Pin-Out-J1962-Explained-What-Is_v2.png" />
    </a>
</p>

| Protocol            | Used Lines         | Implemented | Notes |
|---------------------|--------------------|:-----------:|----------------------------------------------------------------------|
| CAN Bus / ISO 15765 | CAN High & CAN Low | ✓           | Used by almost any modern vehicle. No additional hardware needed; The BB Blue's CAN transceiver can be connected directly to the OBD port's CAN lines. |
| KWP1281             | K Line             | ✓           | Not an official OBD2 protocol, used on older VAG (VW & friends) cars. Returned codes are not official OBD2 DTCs. |
| ISO 9141            | K & L Line         | ✗           | Apparently similar to KWP1281, but an official OBD2 protocol. Not implemented, but technically supported by hardware described below. |
| KWP2000 / ISO 14230 | K & L Line         | (✓)         | Implemented at least partially (K line init only, no fast init). Initialization has been tested, but not actual OBD2 commands (see below). |

There are some more OBD2 protocols, that apparently are mostly used on American vehicles. I have not considered these, and the hardware required seems quite different.

Note that the above is my understanding, this whole field is a giant mess of standards and acronyms.

While I don't have access to any vehicles that uses ISO 9141 or KWP2000, these should be fairly easy to finish, since the K line physical layer is already mostly implemented (except maybe some initialization modifications). If you need one of them and have a vehicle you'd be willing to test on, let me know, I'll gladly help with the implementation.

# Features

- Read Diagnostic Trouble Codes, stored and pending
- Clear Diagnostic Trouble Codes
- Read diagnostic data, current and from freeze frame
- Log diagnostic data to CSV for plotting
- Read and write adaptation values, e.g. to reset the service interval (KWP1281 only, no login functionality _yet_)

# Tested Vehicles/ECUs

| Vehicle/ECU                  | Protocol | Baud/Bit Rate | Notes |
|------------------------------|----------|---------------|-------|
| VW Golf Mk4 (1J), 2001       | KWP1281  | 10400         |       |
| VW Golf Mk7 (AUV), 2014      | CAN      | 500000        | Most things work, but multi-frame messages (VIN, more than 2 DTCs) do not. Might require modifications to flow control messages. |
| ~Renault Zoe (AGVYB0), 2015~ | CAN      | 500000        | CAN bus is active, but does not respond to OBD2 queries. I did not spend much time on this, might require some sort of initialization. |
| Siemens SIMOS2.1B ECU        | KWP1281  | 9600          | [Pinout](https://wiki.obdtuning.de/images/9/99/VAG_Simos2.4.jpg) (also has a 500kbps CAN bus on pins 29 (L) & 31 (H), but this is not used for diagnostics and does not respond to OBD2 queries) |
| Bosch EDC16U34 ECU           | CAN      | 500000        | [Pinout](http://diagprof.com/de/ecu_tipps_und_tricks/vag/motorelektronik/bosch/edc16u34) |
| Bosch EDC16U34 ECU           | KWP2000  | 10400         | Initializes without L line, allows reading VIN, but does not respond to OBD2 queries. |

# Compiling & Usage

You can compile this directly on the BeagleBone, but the compilation times can be annoying, so the project is set up for cross-compilation. You will have to add the `armv7-unknown-linux-gnueabihf` target using `rustup` (You'll also need a version of GCC compiled for `arm-linux-gnueabihf` for linking. Consult your OS' package manager.):

```
$ rustup target add armv7-unknown-linux-gnueabihf
$ cargo build --release
```

Then, copy the executable to the BeagleBone.

See `rustbucket -h` for usage.

# Testing

The program contains modes for testing the hardware and simulating a car so you can test your hardware using two BeagleBones instead of a car. However, the best way to test is to get yourself a used ECU off of eBay. These can be bought relatively cheaply for older cars (I paid less than 20€ for each of the ECUs I tested with). See references below for some ECU pinouts.

<p align="center">
    <a href="https://github.com/KoffeinFlummi/rustbucket/blob/master/doc/test_setup.jpg?raw=true">
        <img width=600 src="https://github.com/KoffeinFlummi/rustbucket/blob/master/doc/test_setup.jpg?raw=true" />
    </a>
</p>

When testing the CAN protocol with two BeagleBone Blues, you should add a terminating resistor (I used 100Ω) between the CAN lines. The R104 120Ω terminating resistor is not populated from the factory on the BB Blue, and with an unterminated CAN bus you will have problems with the higher bit rates (such as the default 500kbps). This may also apply to some desktop ECUs, but the ones I tested with had 66Ω internal resistors.

# Hardware

For the CAN bus, you can simply connect the BeagleBone Blue's CAN lines directly to the OBD2 connector (On the BeagleBone Black, you'll have to connect a CAN transceiver). For the K Line, the 3.3V logic level of the BeagleBone's UART1 bus will have to be converted to the 12V used on the K line, and vice versa.

I tried to implement [this](https://cdn-shop.adafruit.com/datasheets/an97055.pdf) design for a bi-directional logic level converter using a single N-channel MOSFET, but I couldn't figure out how to do so while making sure that the BeagleBone pins don't sink too much current. Since I had a bunch of N-channel MOSFETs lying around anyway, I just went with this horrendously inefficient design using 2 MOSFETs for reading and writing respectively:

<p align="center">
    <a href="https://github.com/KoffeinFlummi/rustbucket/blob/master/doc/schematic.png?raw=true">
        <img width=600 src="https://github.com/KoffeinFlummi/rustbucket/blob/master/doc/schematic.png?raw=true" />
    </a>
</p>

The BeagleBone's UART1 bus (TX: P9.24, RX: P9.26) is used for writing and reading the K line. The schematic also includes the same setup for writing to the L line, but this is not used in software ATM. Q3 has a gate voltage of 12V, the rest is 3.3V.

If you actually know what you're doing electronics-wise, you can probably come up with something better/more efficient. Contributions appreciated.

I used an IRFZ24NPbF for Q3 and IRLB8748PbF for the others since I had them around. If you're buying components specifically for this there are probably better options, and you may want to go with SMD stuff (or just a better design in general). If you are using TO-220 (G-D-S) components like I did, you may also find the terrible PCB I designed useful. I also have some left over, so let me know if you want one.

KiCAD project (schematic + PCB + Gerber files) can be found [here](https://github.com/KoffeinFlummi/rustbucket/tree/master/kicad).

# References

- Marcelo Sacchetin's post on accessing the CAN Bus with the BeagleBone: https://medium.com/bugbountywriteup/car-hacking-with-python-part-1-data-exfiltration-gps-and-obdii-can-bus-69bc6b101fd1
- Description of KWP1281 protocol (German): https://www.blafusel.de/obd/obd2_kw1281.html (The author mentions that the ECU will report at most 4 DTCs per message. I have received up to 7 DTCs in a single message. Not sure what the upper limit is.)
- List of KWP1281 block titles/types: http://nefariousmotorsports.com/forum/index.php?PHPSESSID=r4ifc98vtsa66imq3rnvqtsul1&topic=8274.0title=
- Description of KWP2000 initialization (German): https://www.blafusel.de/obd/vag_kw2000.html
- List of vehicles and their protocol (does not differentiate between the K/KL line protocols): https://www.blafusel.de/obd/vag_compatibility.html
- List of successfully scanned vehicles (German, user-reported and therefore unreliable, but extensive): https://www.blafusel.de/obd/obd2_scanned.php
- List of Fault Codes, including pre-OBD VAG ones: http://wiki.ross-tech.com/wiki/index.php?title=Category:Fault_Codes
- List of ECUs and their pinouts (German, useful for hooking up ECU directly): https://wiki.obdtuning.de/index.php?title=Hauptseite
- More ECU pinouts: http://diagprof.com/ecu_tips_and_tricks/manufacturer-vag/ecudomain-pcm_powertrain_control_module/supplier-bosch
- Some nice examples of CAN bus issues: https://www.kvaser.com/about-can/the-can-protocol/can-oscilloscope-pictures/
