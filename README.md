# I2C Master Controller — MPU-9250

## Overview
Development of standalone I2C master controller in Verilog intended for FPGA-based communication

### Purpose
Establish I2C communication through a hardware-based interface. The controller is designed to operate independently of a CPU, with the eventual goal of integration into a custom RISC-V processor system.
- Development emphasizes implementing I2C communication at the RTL level—including bus timing, finite-state machines (FSMs), bidirectional signaling, command handling, and hardware verification.
- Intended final result is a standalone FPGA subsystem wihtout requiring a RISC-V processor to directly manage I2C signaling.

## Project Objectives
Intended functionality includes:

- Implementing an FPGA-based I2C master supporting 7-bit device addressing.
- Generating START, repeated START, and STOP conditions.
- Transmitting and receiving 8-bit data, including ACK/NACK handling.
- Generating SCL timing from the FPGA clock, initially targeting 100 kHz Standard-mode communication, eventually reaching 400 kHz.
- Implementing SDA/SCL synchronization, clock-stretch handling, and communication timeouts.
- Providing a command/response interface for controlling individual I2C operations.
- Verifying functionality through simulation and physical FPGA testing.

## Hardware and Development Tools

| Component | Purpose |
|---|---|
| Basys 3 FPGA | Hardware platform for implementing and testing the I2C controller |
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
             Sensor
                |
                v
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
- Repeated sensor measurements and data integrity.
- Comparison of simulated behavior with physical waveforms.

## References

- **I2C-bus Specification and User Manual (UM10204)** — NXP Semiconductors.
- **Basys 3 Reference Manual** — Digilent.
- **Vivado Design Suite Documentation** — AMD/Xilinx.