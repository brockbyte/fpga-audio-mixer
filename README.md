# FPGA Digital Audio Mixer with Network Control

## What is this project about?

I’ve been producing music for several years and was always searching for a control surface that offers a flexible number of faders and controls. Since I couldn’t find exactly what I had in mind, I decided to build one myself.
During discussions with another producer, the idea came up to not only design a control surface, but to integrate a basic mixer directly into the device. That was the moment I decided to move forward with a full FPGA-based design.
Initially, I considered combining an FPGA with a separate microcontroller to manage the physical control elements. However, because I wanted to display real-time information — such as peak meters — with fully deterministic timing, I chose a pure FPGA implementation in Verilog.
This approach provides completely predictable timing behavior. For example, the displays refresh every 20 ms — entirely independent of any other processes running on the chip. Additionally, the FPGA offers enough I/O pins to directly connect more complex control elements like rotary encoders. I can also instantiate as many I²C controllers as the available logic resources allow.

### Mixer Architecture

One of the key design questions was the type of input the mixer should support.
I initially experimented with implementing the Behringer UltraNet protocol (based on an existing open-source project). However, I wasn’t satisfied with the stability of the data transmission on my development board.
Since I had already worked extensively with FPGA-based networking in the past — and was interested in exploring DANTE/AES67 technologies — I decided to design the mixer around network-based audio streams.
While clock jitter still needs careful consideration, handling it within an FPGA environment feels more transparent and easier to debug compared to proprietary protocols. Thanks to DANTE’s AES67 mode, the mixer integrates seamlessly with professional studio equipment.
At the same time, you don’t need expensive studio hardware to experiment with the system. There are several open-source AES67-compatible software tools available, making it accessible for development and testing.

### Project Overview

This project combines:
* A fully FPGA-based digital mixer
* Motorized faders and hardware controls
* Deterministic display updates and metering
* Network-based audio input (AES67-compatible)
* Flexible and scalable architecture

If you have any questions, feel free to reach out.
I also have a limited number of PCBs from the first production run available — in case you’d like to start soldering and build your own board.

![System Picture](images/Mixerboard.jpg)

**Table of Contents**
1. [Hardware Components](#hardware-components)
2. [System Overview](#system-overview)
3. [Architecture & Clock Domains](#architecture--clock-domains)
4. [Detailed Module Specifications](#detailed-module-specifications)
5. [Signal Formats & Data Paths](#signal-formats--data-paths)
6. [Configuration Parameters](#configuration-parameters)

---

## Hardware Components

### Intel Cyclone 10 LP Development Board
The heart of the digital audio mixer is the Intel Cyclone 10 LP Evaluation Kit. This platform was selected as it provides a few significant benefits:

* sufficient large Cyclone 10 FPGA to implement network, audio and control functionality
* ingegrated 1G network port
* a large number of input / output pins
* on-board JTAG programmer and debugger
* acceptable price of around 100 EUR

### Control Board Bill of Materials (BOM)
The custom control surface communicates with the FPGA via a 40-pin GPIO ribbon cable and some more dedicated cables. The board has been designed with KiCad. For the first production batch, I did not take special care of analog signal routing for the ADC for the faders and touch. It works in my environment, however, for the next production run maybe someone with more experience on that could give some hints on the layout ;-)

For this design I selected 60 mm faders, as the board would have become too big with the 100 mm faders. I tried to avoid using SMD components in this early design phase to make manual corrections easy and only used pre-built modules that can be either stacked into sockets or soldered directly.

| Component | Part Number | Quantity | Function |
| :--- | :--- | :--- | :--- |
| **Motorized Faders** | **MF60T** | 8 | Behringer 60 mm standard 10k Linear |
| **Motor Drivers** | **TB6612** (Dual) | 4 | PWM-based motor driver for 8 fader motors |
| **Touch Sensor** | **MPR121** | 1 | Capacitive touch detection |
| **I2C Multiplexer** | **PCA9548** | 1 | Individual addressing for the 8 OLED displays |
| **OLED Displays** | **128x64 I2C** | 8 | Per-channel labels, Pan, and Peak Meters |
| **Port Expander** | **MCP23017** | 1 | Driving LEDs for Mute and Solo buttons |
| **Rotary Encoders** | **STEC12** | 8 | Pan control |
| **Connectors** | 40-pin IDC | 1 | Main interface to the FPGA development board |

---

## Network & Protocol Implementation

### AES67 & SDP
The mixer adheres to the **AES67 standard** for interoperability with professional Audio-over-IP (AoIP) ecosystems.

* **Fixed Parameters:** To maintain a pure hardware implementation, the stream is fixed at **48 kHz, 24-bit (L24 format)**.
* **Session Description Protocol (SDP):** A static SDP file is provided to define multicast addresses, ports, and payload types, allowing external receivers to subscribe to the mixer's output.

### Multicast & Switch Strategy
The system utilizes **IP Multicast** for efficient one-to-many audio distribution.

* **Simplified Implementation:** To reduce FPGA gate usage and complexity, the design **does not implement IGMP Join/Leave messages**. 
* **Network Requirement:** Because the hardware does not actively "announce" its presence via IGMP, the network switch must be configured for **Static Multicast** or allowed to flood the multicast traffic to the target ports. This trade-off allows for a significantly leaner RTL network stack and is absolutely sufficient for hoem or small studio environments.

### Network Configuration

Since this project implements a pure hardware UDP/IP stack, IP addresses are currently **hardcoded** in the RTL to ensure maximum performance and zero-config startup.

**Current Default Settings**
My DANTE network has the IP range 192.168.1.0/24. By default, the project is configured with the following static IP addresses:

| Parameter | Default Value | Description |
| :--- | :--- | :--- |
| **Mixer IP Address** | `192.168.1.128` | The static IP of the FPGA board |
| **Frontend Server IP Address** | `192.168.1.100` | The static IP of the python server that receives the status messages |
| **Subnet Mask** | `255.255.255.0` | Standard Class C subnet |
| **Gateway** | `192.168.1.1` | Default network gateway |
| **Multicast Group** | `239.69.1.2` | Destination for AES67 RTP audio streams |
| **RTP Port** | `5004` | Standard port for RTP audio |

**How to Change IP Addresses**
If your network environment uses a different subnet, you must update the addresses in the source code before recompiling in Quartus.

1. **FPGA RTL (Hardware):**
   Look for the hexadecimal representation of the IP:
   * Example: `32'hC0A80164` corresponds to `192.168.1.100`.
   
2. **Web Frontend:**
   Open the `frontend/config.js` (or similar configuration file in the `/frontend` directory).
   Update the `MIXER_IP` constant to match the address set in the FPGA:
   ```javascript
   const MIXER_IP = "192.168.1.128";

## System Overview

### Board

The following figure shows the elements on the custom mixer control baord. The control board is powered with 3,3V from the FPGA board and has an additional 9V power supply for the fader motors.

**Board Picture**

![Board Drawing](images/board_drawing.png)

#### 1. Fader

Each channel features a motorized linear fader at the bottom of the panel.
It is used to control the channel level (volume). The motorization allows the fader to move automatically to reflect external control from the web interface.

#### 2. Mute / Solo Buttons

Above each fader there are illuminated push buttons:

- Mute: Instantly silences the corresponding channel when activated.
- Solo: Isolates the channel for monitoring, muting all other channels depending on the solo mode.

The LED backlighting provides immediate visual feedback about the current state (active/inactive).

#### 3. OLED Display

Each channel has a small OLED display positioned above the buttons.
The display shows channel-related information, such as:

- Channel or track name
- Pan position (L / C / R)
- Pre-fader input meter

#### 4. Pan Rotary Knob

At the top of each channel strip there is a rotary encoder (pan knob).
It is used to adjust the stereo pan position, placing the signal anywhere between the left and right output channels.
Because it is an encoder rather than a simple potentiometer, it supports precise digital control and remote control.

#### 5. Rotary connector

Initially I intended to use port extenders for the rotary encoders. 
However, during development it turned out that the extenders are too slow for quick movements of the encoders and debouncing was hard.
So I changed the setup and connected the encoders directly to the FPGA, which allows flexible debounce and stable and quick detection. 

#### 6. MCP23017 port extenders

The port extenders are used for the LED buttons.

#### 7. Main PCB connector

A 40 pin ribbon cable connects the board to the FPGA system.

#### 8. TB6612 motor drivers

In total 4 motor drivers are on the board for the 8 fader channels.

#### 9. MPR121 capacitative touch

The touch IC detects if a fader knob is touched and motor control is disabled for this channel.

#### 10. PCA9548 I2C multiplexer

The I2C multiplexer is used for the 8 OLED displays.

### Web frontend

![Web Frontend](images/web_frontend.png)

The web frontend implements the same controls that are available on the local control interface.
It can sends updates to the mixer and receives regular status updates from the FPGA main mixer system.

### Core Subsystems

**Audio Subsystem**
- 16-channel input mixer with independent gain, pan, mute, and solo per channel (current demo system uses 8 channels)
- 2-channel stereo output (summed from 16 inputs with pan law)
- AES67 RTP network audio receive (UDP port 5004)
- Optional network audio transmit capability
- Adaptive jitter buffer with clock drift compensation
- Peak detection and hold/decay metering

**Timing Subsystem**
- IEEE 1588 (PTP v2) clock synchronization to master device
- Automatic 48 kHz audio clock generation locked to master timing
- PI servo controller for precise frequency tracking
- Anti-windup and wrap-safe counter management

**Network & Telemetry**
- UDP status packet streaming (port 7880) with complete mixer state snapshot
- Packet includes peaks, pan, gain, mute/solo, channel names, jitter buffer status
- Optional SDP (Session Description Protocol) announcements for device discovery

### Included Libraries

#### verilog-ethernet

The project relies on the UDP / Ethernet implementation made by Alex Forencich.
It is available here: https://github.com/alexforencich/verilog-ethernet

#### verilog-i2c

For the I2C communication, the i2c-master from verilog-12c by Alex Forencich is used.
It is available here: https://github.com/alexforencich/verilog-i2c/tree/master

#### spdif

A working SPDIF transmitter, which I have tried with this project can be found here:
https://ackspace.nl/wiki/SP/DIF_transmitter_project

---

## Architecture & Clock Domains

### Multi-Clock System Design

The FPGA uses three synchronized but independent clock domains:

```
┌─────────────────────────────────────────────────────────────┐
│                    Ethernet PHY (1000BASE-T)                │
│                         RGMII Interface                     │
└────────────────────────────┬────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ↓                    ↓                    ↓
    125 MHz             50 MHz Primary       12.288 MHz SPDIF
  Ethernet Clock      Control Clock          Output Clock
    (Network)          (Mixed Signal)         (Audio Output)
        │                    │                    │
        ├─ UDP/IP Stack      ├─ Audio Mixer      └─ SPDIF Output
        ├─ RTP Parsing       ├─ Fader Control       Encoding
        ├─ PTP Reception     ├─ Button Input
        ├─ Packet Routing    ├─ OLED Display
        └─ Network Timing    └─ Status Capture
                             
```

### Clock Domain Crossing Strategy

**Safe data transfer between domains:**

| Crossing | Method | Module | Purpose |
|---------|--------|--------|---------|
| 125 MHz → 50 MHz | Async FIFO | `axis_udp_payload_async_fifo.v` | UDP packet payload buffering |
| 50 MHz → 125 MHz | Async FIFO | `tx_asyncfifo10b_axi.v` | Network transmit path |
| 48 kHz ↔ 50 MHz | Custom FIFO | `jitter_buffer_dcfifo_resampler.v` | Audio buffering with resampling |

### System Hierarchy

```
fpga.v (Top-Level Wrapper)
├── PLL (generates 50M, 12.288M from 125M input)
├── Reset Synchronizers (async reset → sync reset per domain)
│
└── fpga_core.v (Main Integration)
    │
    ├── Ethernet MAC Layer (verilog-ethernet library)
    │   ├── RGMII RX/TX (125 MHz physical interface)
    │   ├── Ethernet MAC (address filtering, CRC)
    │   └── ARP and basic management
    │
    ├── IP & UDP Stack (verilog-ethernet library)
    │   ├── IPv4 header processing
    │   ├── UDP demultiplexer (4 ports: 319, 320, 5004, 7880)
    │   ├── UDP transmit aggregator
    │   └── Checksum generation
    │
    ├── PTP Client Implementation
    │   ├── ptp_client_stream_50m: Sync message reception
    │   ├── ptp_v2_sync_follow_parser_stream: Timestamp extraction
    │   ├── ptp_client_servo_light_wrapsafe_dbg: PI servo (KP=0.5, KI=0.001)
    │   └── ptp_timebase_slew: 48 kHz generation with slew-rate limiting
    │
    ├── Audio Processing Path (50 MHz → 48 kHz)
    │   ├── aes67_rtp_rx_fifo: UDP → RTP packet parsing
    │   ├── jitter_buffer_dcfifo_resampler: Adaptive buffering + resampling
    │   ├── mixer_16x2_q14_pipelined_patched: 3-stage pipelined 16×2 mixer
    │   ├── pan_gain_coeff_calc16_rom: Pan law coefficient lookup (256-entry ROM)
    │   └── aes67_tx_stream2_from_fifo48_lr: RTP packet generation (optional output)
    │
    ├── Control Interface Subsystem (50 MHz)
    │   │
    │   ├── Fader Motor Control
    │   │   ├── mcp3008_rr: 8-channel SPI ADC round-robin scanner (100 kHz SPI)
    │   │   ├── fader_channel_logic_10b_int: Per-channel PI servo (Kp=0.5, Ki=0.03)
    │   │   └── mixer_top_8ch: Orchestrator (PWM @ 20 kHz to TB6612 drivers)
    │   │
    │   ├── Button Input (16 buttons via 2× MCP23017)
    │   │   ├── mcp23017_dual_ctrl: I2C GPIO read (2 ms poll interval)
    │   │   └── mcp_buttons_toggle_plus_solo: Debounce + toggle/solo logic
    │   │
    │   ├── Touch Sensing (8 sensors via MPR121)
    │   │   └── mpr121_poll8_simplified-v2: I2C polling (1 ms interval)
    │   │
    │   └── Rotary Encoders (8 channels, quadrature decoding)
    │       └── rotary8_stec12e08_debounced: Quadrature phase detection
    │
    ├── Display & Monitoring System (50 MHz)
    │   ├── oled_mixer8_ssd1306_tca9548_axis: 8-display controller (20 ms refresh)
    │   │   ├── TCA9548 I2C multiplexer (1-of-8 channel selection)
    │   │   ├── Name
    │   │   ├── Peak meter rendering (12 segments, hold/decay)
    │   │   └── Pan indicator visualization
    │   │
    │   ├── name_ram_1w2r: 1-write, 2-read BRAM (256×8, dual independent reads)
    │   ├── name_dp_ram: Dual-port BRAM building block
    │   └── font7x12_pages_bram: Font storage (1536 bytes, M9K BRAM)
    │
    ├── Status & Telemetry
    │   ├── udp_status_tx_bram: Status capture (424-byte snapshot per cycle)
    │   ├── sdp_udp_stream_from_rom: SDP announcer (periodic broadcast)
    │   └── udp_debug_tx: Optional debug streaming
    │
    ├── Network Routing
    │   ├── udp_port_demux_4: 4-way input routing (fixed priority)
    │   └── udp_tx_mux4_from_streams_clean: 4-way output arbitration (priority 0>1>2>3)
    │
    └── Clock Domain FIFOs
        ├── axis_udp_payload_async_fifo: 8-bit, 512 deep (125M→50M crossing)
        ├── tx_asyncfifo10b_axi: 10-bit, 4096 deep (50M→125M crossing)
        ├── custom_async_fifo: 48-bit, 512 deep (general purpose)
        ├── net_async_fifo_10b: 10-bit, 4096 deep (packet format {start,end,data})
        ├── aes67sendfifo: 50-bit, 256 deep (audio tx buffering)
        └── sdebug_async_fifo: 8-bit, 4096 deep (debug streaming)
```

