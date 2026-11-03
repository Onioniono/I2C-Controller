# I2C Master Controller — MPU-9250

## Overview
Development of standalone I2C master controller in Verilog, intended for FPGA-based communication with the MPU-9250 inertial measurement unit (IMU).

### Purpose
Establish I2C communication, configure the MPU-9250, and acquire accelerometer and gyroscope measurements through a hardware-based interface. The controller is designed to operate independently of a CPU, with the eventual goal of integration into a custom RISC-V processor system.
- Development emphasizes understanding and implementing I2C communication at the RTL level—including bus timing, finite-state machines (FSMs), bidirectional signaling, command handling, and hardware verification.
- Intended final result is a standalone FPGA subsystem capable of repeatedly acquiring MPU-9250 measurements wihtout requiring a RISC-V processor to directly manage I2C signaling.

## Project Objectives
Intended functionality includes:

- Implementing an FPGA-based I2C master supporting 7-bit device addressing.
- Generating START, repeated START, and STOP conditions.
- Transmitting and receiving 8-bit data, including ACK/NACK handling.
- Generating SCL timing from the FPGA clock, initially targeting 100 kHz Standard-mode communication, eventually reaching 400 kHz.
- Implementing SDA/SCL synchronization, clock-stretch handling, and communication timeouts.
- Providing a command/response interface for controlling individual I2C operations.
- Initializing and configuring the MPU-9250 through its internal registers.
- Reading accelerometer and gyroscope measurements through multi-byte transactions.
- Reconstructing sensor data into signed 16-bit values.
- Verifying functionality through simulation and physical FPGA testing.

## Hardware and Development Tools

| Component | Purpose |
|---|---|
| Basys 3 FPGA | Hardware platform for implementing and testing the I2C controller |
| MPU-9250 (HiLetgo breakout) | Accelerometer and gyroscope sensor |
| Verilog HDL | RTL design and hardware behavior |
| Visual Studio Code | Source code development |
| AMD/Xilinx Vivado | Simulation, synthesis, implementation, and FPGA programming |
| Logic analyzer / oscilloscope | Physical SDA/SCL waveform verification |

SDA and SCL use open-drain signaling, requiring pull-up resistors and compatible I/O voltage levels. Physical connections, breakout-board power requirements, and FPGA pin assignments must be verified before hardware testing.

## System Architecture

**Standalone architecture:**

```text
Sensor Command Sequencer / Testbench
                |
                | Command / Response
                v
        I2C Master Controller
        +-------------------+
        | Command Handling  |
        | FSM / Timing      |
        | TX / RX Registers |
        | ACK / NACK Logic  |
        | SDA/SCL Control   |
        +-------------------+
                |
                | SDA / SCL
                v
             MPU-9250
                |
                v
     Accelerometer / Gyroscope
              Data
```

### I2C Master Controller

The `i2c_master.v` module is intended to provide reusable, byte-oriented I2C operations through four commands:

| Command | Function |
|---|---|
| `CMD_START` | Generate an initial or repeated START condition |
| `CMD_STOP` | Generate a STOP condition and terminate the transaction |
| `CMD_WRITE` | Transmit one 8-bit value and receive the slave's ACK/NACK |
| `CMD_READ` | Receive one 8-bit value and transmit ACK or NACK |

- The controller uses a ready/valid handshake interface to accept commands and return responses.
- Individual commands form larger I2C transactions, with the bus remaining active between commands until STOP is issued.
- The main FSM handles operations such as START, WRITE, READ, ACK/NACK, STOP, and response management. 
- Internal timing phases and counters control SDA/SCL transitions according to I2C timing requirements.

## MPU-9250 Communication

- The MPU-9250 combines a three-axis accelerometer, three-axis gyroscope, and three-axis magnetometer.
- The initial application focuses on accelerometer and gyroscope measurements. 
- Magnetometer communication is outside the primary acquisition requirements.

### Sensor Addressing and Configuration

The MPU-9250 uses a 7-bit I2C address:

- `0x68` when AD0 is LOW.
- `0x69` when AD0 is HIGH.

Relevant internal registers include:

| Register | Address | Purpose |
|---|---|---|
| `WHO_AM_I` | `0x75` | Verify sensor identity |
| `PWR_MGMT_1` | `0x6B` | Power and clock configuration |
| `PWR_MGMT_2` | `0x6C` | Accelerometer/gyroscope axis control |
| `SMPLRT_DIV` | `0x19` | Sample-rate configuration |
| `CONFIG` | `0x1A` | Gyroscope filtering |
| `GYRO_CONFIG` | `0x1B` | Gyroscope full-scale range |
| `ACCEL_CONFIG` | `0x1C` | Accelerometer full-scale range |
| `ACCEL_CONFIG2` | `0x1D` | Accelerometer filtering |
| `ACCEL_XOUT_H` | `0x3B` | Beginning of accelerometer data |
| `GYRO_XOUT_H` | `0x43` | Beginning of gyroscope data |

### Register Read Sequence

Typical register read follows this transaction:

```text
START
  |
  +-- Device Address + WRITE
  |       |
  |      ACK
  |
  +-- Register Address
  |       |
  |      ACK
  |
REPEATED START
  |
  +-- Device Address + READ
  |       |
  |      ACK
  |
  +-- Receive Data Byte(s)
  |       |
  |      ACK (continue reading)
  |      NACK (final byte)
  |
STOP
```

### Accelerometer and Gyroscope Acquisition

The intended acquisition method uses a 14-byte burst read beginning at register `0x3B`.

| Register range | Data |
|---|---|
| `0x3B–0x40` | Accelerometer X, Y, Z |
| `0x41–0x42` | Temperature |
| `0x43–0x48` | Gyroscope X, Y, Z |

- Each accelerometer and gyroscope axis consists of two bytes, which are combined into a signed 16-bit measurement.
- The burst read uses ACK for the first 13 received bytes, followed by NACK for the final byte and a STOP condition.
- Temperature bytes may be received without being used in the primary sensor output. 
- The intended result is a complete set of six raw signed measurements suitable for subsequent processing.

## Verification and Testing

Verification is intended to cover both simulated behavior and physical FPGA communication.

**Simulation (Vivado):**
- Clock and synchronous reset behavior.
- Command/response handshake correctness.
- START, repeated START, and STOP generation.
- SDA/SCL timing and data setup/hold requirements.
- Byte transmission and reception.
- ACK/NACK handling.
- Multi-byte transaction sequencing.
- Clock stretching, timeout behavior, and error handling.
- Stability of received data and response signals.

**Hardware validation (Basys 3):**
- SDA/SCL electrical configuration and pull-up verification.
- I2C signal inspection using a logic analyzer or oscilloscope.
- MPU-9250 device identification through `WHO_AM_I`.
- Sensor configuration register access.
- Accelerometer and gyroscope data acquisition.
- Repeated sensor measurements and data integrity.
- Comparison of simulated behavior with physical waveforms.

## Future RISC-V CPU Integration

Although the controller is developed independently in this repository, it is intended to eventually operate as a hardware peripheral alongside a custom RISC-V CPU.

CPU integration will take place within the group's separate RISC-V repository rather than this standalone project.

### Intended Integration Architecture

```text
RISC-V Software
      |
      | LW / SW Instructions
      v
RISC-V CPU
      |
      v
Memory-Mapped I/O (MMIO)
Address Decoder
      |
      v
I2C Peripheral Registers
      |
      v
Command / Response Interface
      |
      v
I2C Master Controller
      |
      | SDA / SCL
      v
MPU-9250
```

### CPU and Peripheral Requirements

Integration will require coordination between the CPU datapath, memory interface, and I2C hardware:

| Requirement | Description |
|---|---|
| MMIO address decoding | Assign an address range to the I2C peripheral, separate from RAM |
| CPU load/store access | Route RISC-V `LW` and `SW` operations to the appropriate peripheral registers |
| Control registers | Provide software-accessible commands and configuration |
| Status registers | Expose busy, completion, ACK/NACK, and error information |
| TX/RX data registers | Transfer outgoing bytes and received sensor data |
| Command interface adapter | Translate register accesses into the I2C master's ready/valid command interface |
| Sensor data buffering | Preserve received measurements for CPU access |
| Timing coordination | Ensure compatibility between CPU memory accesses and peripheral operations |
| Software interface | Provide routines for initialization, register access, and sensor sampling |

- A possible MMIO register layout may include `CONTROL`, `STATUS`, `SLAVE_ADDRESS`, `REGISTER_ADDRESS`, `TX_DATA`, and `RX_DATA`. 
- The exact addresses and register behavior will be determined during integration.
- The CPU will not need to directly generate SDA/SCL transitions. Instead, it will issue commands through memory-mapped registers, while the I2C hardware handles the communication sequence and reports the result.

## Scope and Project Boundaries

This repository covers standalone I2C controller development, MPU-9250 communication, and independent verification.

The following related work belongs to the wider design system rather than this repository:

- Direct integration with the RISC-V CPU datapath and memory system.
- MMIO address decoding and CPU-facing register implementation.
- CPU software for sensor processing and control.
- Accelerometer/gyroscope sensor fusion and orientation estimation.
- PID or other feedback-control algorithms.
- PWM generation and actuator control.

The standalone controller and sensor acquisition logic are intended to serve as a reusable hardware foundation for those higher-level applications.

## References

- **MPU-9250 Product Specification** — InvenSense, PS-MPU-9250A-01, Revision 1.1.
- **MPU-9250 Register Map and Descriptions** — Sensor register configuration and addressing.
- **I2C-bus Specification and User Manual (UM10204)** — NXP Semiconductors.
- **Basys 3 Reference Manual** — Digilent.
- **Vivado Design Suite Documentation** — AMD/Xilinx.