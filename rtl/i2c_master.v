/* *********************************************************
Name: Alexander Mendoza
Role: Sensor Integration
Creation Date: 10-27-2026

File Name: i2c_master.v
Purpose: I2C Master Module for RISCV Peripherals
Revision History:
- 10-27-2026 — Initial skeleton implementation; Establish command/response framework; Add a commented FSM outline
- 10-28-2026 — Add Reset; Add I2C Clock Timing
- 11-02-2026 — Add SDA/SCL Synchronization; Adjust FSM phases & timing
- 11-06-2026 — Add FSM logic for: IDLE, START, SLAVE_ACK, MASTER_ACK, RESPONSE
- 11-07-2026 — Add FSM logic for: WRITE, READ, STOP, ERROR, default
- 11-08-2026 — Add timeout logic
********************************************************* */

// --------------------------------------------------------------------------------
// I2C Master Module
/*
Purpose:
- Generate the SDA/SCL sequence for START, STOP, writing one byte, and reading one byte. Another controller chooses the order of these commands.
- That controller can later connect to sensor acquisition or CPU/MMIO logic.
- One command is one operation; a transaction is the sequence from START through STOP. 
- Finishing a byte does not automatically finish the transaction.
Notes:
* Many parameters ard registers are possibilities; some may end up unused for now.
1. Accept a command on a clk rising edge with cmd_valid && cmd_ready.
  - Until that edge, the caller holds cmd_valid and its arguments stable.
2. Capture those arguments, execute one command, then present one response.
  - Do not accept another command until that response has been consumed.
3. Keep rsp_valid and the response fields stable until a rising edge with rsp_valid && rsp_ready. 
  - This lets the receiver take time to collect it.
4. START begins a transaction. START during an active transaction produces repeated START. READ/WRITE/STOP require an active transaction.
5. Between commands within a transaction, hold SCL LOW. Preserve SDA until it can be changed safely; waiting for the caller extends the LOW interval.
6. Slave NACK is reported to the caller, which decides what to do next, normally STOP. Master NACK after the final received byte is normal.
7. Reset releases both pins and clears local state. It aborts local work; it does not guarantee a valid STOP or recovery of the external device.
- Single-master operation, initially 100 kHz.
- Plan configurable timing, clock-stretch waiting, and bounded timeouts before considering it complete.
- Multi-master arbitration, ten-bit addressing, and automatic bus recovery are outside the initial implementation scope.
*/
// --------------------------------------------------------------------------------
// ******************************************************* PARAMETERS OUTLINE *******************************************************
module i2c_master #(
  /* Notes:
  Parameters are configuration constants not runtime input signals
  100 MHz is a default only (set CLK_HZ to the actual FPGA input clock)
  100_000 = 100000; notation is for readability (functions the same)
  */
  parameter integer CLK_HZ = 100_000_000,
  parameter integer I2C_HZ = 100_000,
  // Maximum duration of one accepted command (including clock stretching)
  parameter integer TIMEOUT_MS = 1
)(
  input wire clk,   // Clock for every internal always block
  input wire rst,   // Planned active-high synchronous reset

  // Commands INTO this module (these are FPGA-side wires)
  input wire cmd_valid,         // Caller: "I am presenting a command"
  output reg cmd_ready,         // Master: "I can accept that command"
  input wire [1:0] cmd_opcode,  // Select START, STOP, WRITE, or READ below
  input wire [7:0] cmd_data,    // Byte to send for WRITE (ignored otherwise)
  input wire cmd_nack,          // READ: 0 = send ACK; 1 = send final-byte NACK

  // Responses OUT of this module (not a physical I2C wire)
  output reg rsp_valid,         // Master: "A command result is available"
  input wire rsp_ready,         // Caller: "I can collect that result"
  output reg [7:0] rsp_data,    // Completed READ byte (zero for other commands)
  output reg rsp_nack,          // WRITE: 1 = slave did not ACK (zero otherwise)
  output reg [1:0] rsp_error,   // Command/bus fault code defined below

  output reg bus_active,   // Local transaction started, not yet stopped
  inout wire sda,          // Actual shared data pin: transmit & receive
  inout wire scl           // Actual clock pin: pull LOW or release
);

  // COMMAND AND ERROR NAMES
    /* Notes:
    localparam gives a fixed name to a constant (similar to a named C constant).
    [1:0] means two bits
    2'd2 means "decimal 2 represented using two bits".
    Caller is the module that uses this I2C master (e.g. a CPU or sensor interface).
    */
  // A future caller must use the same command encoding (no CPU addresses here)
  localparam [1:0] CMD_START = 2'd0;
  localparam [1:0] CMD_STOP = 2'd1;
  localparam [1:0] CMD_WRITE = 2'd2;
  localparam [1:0] CMD_READ = 2'd3;
    // ***Hypothetical TBD return errors based on internet research & I2C protocol***
  localparam [1:0] ERR_NONE = 2'd0;
  localparam [1:0] ERR_COMMAND = 2'd1;  // Operation invalid in current bus state
  localparam [1:0] ERR_TIMEOUT = 2'd2;  // Wait exceeded a configured time limit
  localparam [1:0] ERR_BUS = 2'd3;      // Bus unavailable or unexpected pin state
  // rsp_nack is separate: a slave NACK is not a timeout or command error

  // REQUEST/RESPONSE HANDSHAKES
    /* Notes:
    These expressions mean "a transfer takes place on the next rising edge".
    Both sides use clk to synchronize "handshake".
    A "handshake" is a two-way signal exchange: one side says "I have data" and the other side says "I can accept it".
    */
  wire command_accept;
  wire response_accept;
  assign command_accept = cmd_valid && cmd_ready;   // Command is valid and the master is ready to accept it
  assign response_accept = rsp_valid && rsp_ready;  // Response is valid and the caller is ready to accept it

  // INTERNAL STORAGE
    /* Notes:
    reg allows assignment inside an always block:
      - It does not mean a physical register.
      - Assigning it in a clocked always block describes flip-flops that retain a value between clock edges.
    */
  reg [1:0] opcode_latched;     // Command copied at acceptance, then held
  reg read_nack_latched;        // Hold the caller's choice through a READ
  reg [7:0] tx_shift_reg;       // Eight-bit byte being transmitted
  reg [7:0] rx_shift_reg;       // Assemble eight received bits into one byte
  reg [2:0] bit_index;          // Three bits represent positions 0 through 7

        /* Notes:
        Address and data share the same byte transmitter:
        - Device address byte: {seven_bit_address, read_bit}; 0 = write, 1 = read
        - Sensor register pointer: an ordinary byte, meaningful to that sensor
        - Payload: another ordinary byte. Received payloads are also EIGHT bits
        The braces above concatenate bits: seven address bits + R/W = eight bits.
        ACK/NACK occupies a separate ninth clock, not part of the eight-bit byte.
        No separate ADDR or R/W state is needed in this reusable byte engine.
        */

        /* Notes:
        rsp_data is planned one-byte result storage. Keep it unchanged while rsp_valid waits for rsp_ready.
        A sensor-sample buffer belongs outside.
        */

// ******************************************************* SDA/SCL SYNCHRONIZATION OUTLINE *******************************************************
  // PHYSICAL OPEN-DRAIN CONNECTIONS
    /* Notes:
    These flags describe what OUR output driver does (not the observed level)
    1 = pull LOW; 0 = disconnect/release

    1'bz means high impedance.
    Pull-ups produce HIGH when every device releases the line.
    Tri-state drivers belong at FPGA I/O pins; connect these ports to the physical pins.
    */
  reg sda_drive_low;
  reg scl_drive_low;
  assign sda = sda_drive_low ? 1'b0 : 1'bz;
  assign scl = scl_drive_low ? 1'b0 : 1'bz;

  // OBSERVING THE EXTERNAL PINS
    /* Notes:
    External SDA/SCL changes need not align with clk.
    Two clocked stages reduce the chance of an unstable input propagating into the FSM.
    Synchronization adds delay; later timing logic must account for it.

    Reminder: Releasing SCL is not proof it is HIGH
    - it takes time to rise, and a slave may hold it LOW (clock stretching)
    - Wait for scl_sync, with a timeout
    Metastability: May fail to identify a HIGH/LOW if input changes near the sampling edge.
    - Introduce a tw-stage synchronizer
    - sda/scl_meta is the first stage
    - sda/scl_sync is the second stage that the FSM observes
    */
  reg sda_meta;
  reg sda_sync;
  reg scl_meta;
  reg scl_sync;

// SYNCHRONIZATION LOGC
always @(posedge clk) begin
  if (rst) begin
    sda_meta <= 1'b1;
    sda_sync <= 1'b1;
    scl_meta <= 1'b1;
    scl_sync <= 1'b1;
  end else begin
    sda_meta <= sda;
    sda_sync <= sda_meta;
    scl_meta <= scl;
    scl_sync <= scl_meta;
  end
end

// ******************************************************* FSM OUTLINE *******************************************************
  // FSM FRAMEWORK: BROAD STATES
    /* Notes:
    START / ADDR / R-W / ACK / DATA / STOP sequence describes a complete transaction.
    Here WRITE handles an address byte OR any other outgoing byte.
    Distinguish receiving the slave's ACK from sending our ACK after a read.

    These are broad state groups, not a finished timing design.
    Each bus operation needs LOW/setup, HIGH/wait/sample, and LOW/finish phases.
    Later choose a phase counter or explicit substates for those intervals.
    cmd_ready being HIGH is not the same as the physical bus being idle:
    - between bytes the master may accept a command while bus_active is HIGH
    */
  localparam [3:0] ST_IDLE = 4'd0;
  localparam [3:0] ST_START = 4'd1;       // Also handles repeated START
  localparam [3:0] ST_WRITE = 4'd2;
  localparam [3:0] ST_SLAVE_ACK = 4'd3;   // Reads an acknowledgement (after ST_WRITE)
  localparam [3:0] ST_READ = 4'd4;
  localparam [3:0] ST_MASTER_ACK = 4'd5;  // Sends an acknowledgement (after ST_READ)
  localparam [3:0] ST_STOP = 4'd6;
  localparam [3:0] ST_RESPONSE = 4'd7;
  localparam [3:0] ST_ERROR = 4'd8;
  reg [3:0] state;

  // FSM FRAMEWORK: TIMING PHASES
    /* Notes:
    Temporary mini-states within each FSM state describe the timing intervals.
    - Currently the phases are not final decisions but possible placeholders and suggestions.
    Not every phase necessarily requires a timing counter
    - Some phases wait for an external condition (e.g. scl_sync HIGH)
    */
    // Reusable Phases
  localparam [2:0] PH_SETUP = 3'd0;
  localparam [2:0] PH_WAIT_HIGH = 3'd1;
  localparam [2:0] PH_HIGH = 3'd2;
  localparam [2:0] PH_LOW = 3'd3;
    //Specialized Phases
  localparam [2:0] PH_START_PREPARE = 3'd4;
  localparam [2:0] PH_START_HOLD = 3'd5;
  localparam [2:0] PH_STOP_RELEASE = 3'd6;
  reg[2:0] phase;

  // TIMING REQUIREMENTS
    /* Notes:
    Uses placeholder clock frequency during isolation design.
    - Later replace with FPGA/RISCV clock frequency.
    The I2C specification defines minimum setup and hold times for START, STOP, and data bits.
    - These are the minimum times that SDA must be stable before and after SCL transitions.
    1_000_000 comes from converting seconds to microseconds
    */
    // Timing intervals for special I2C conditions
  localparam integer START_HOLD_CYCLES = (CLK_HZ * 4.0)/1_000_000;  // >= 4.0 us
  localparam integer START_SETUP_CYCLES = (CLK_HZ * 4.7)/1_000_000; // >= 4.7 us
  localparam integer STOP_SETUP_CYCLES = (CLK_HZ * 4.0)/1_000_000;  // >= 4.0 us
  localparam integer DATA_SETUP_CYCLES = (CLK_HZ * 0.25)/1_000_000; // >= 0.25 us
    // Timing intervals for repeated data-bit clocking
  localparam integer SCL_HIGH_CYCLES = (CLK_HZ * 4.0)/1_000_000;  // >= 4.0 us
  localparam integer SCL_LOW_CYCLES = (CLK_HZ * 4.7)/1_000_000;   // >= 4.7 us
    // Command timeout; independent of I2C bit/phase timing (Round up to clk cycles)
  localparam [63:0] TIMEOUT_CYCLES = (CLK_HZ > 0 && TIMEOUT_MS > 0) ? ((64'd1 * CLK_HZ * TIMEOUT_MS + 64'd999) / 64'd1000) : 64'd1;
  localparam integer TIMEOUT_WIDTH = (TIMEOUT_CYCLES > 1) ? $clog2(TIMEOUT_CYCLES) : 1;
    // Counters
  reg [9:0] clk_count;
  reg [TIMEOUT_WIDTH-1:0] timeout_count;
  reg command_finished;

  // CLOCKED LOGIC OUTLINE
    /* Notes:
    always - describes hardware that reacts on EVERY clk rising edge.
    case - selects the current state, similar in syntax to a C switch.

    One state can last many clock cycles. 
    Counters decide when it advances: no software loop, delay statement, or CPU instruction is needed per bit.
    Use <= for clocked updates: right-hand sides use values BEFORE the edge.
    Phases counter act as mini-states within each FSM state; however, they are not a separate FSM.

    clk_count determines WHEN SDA/SCL change occurs.
    FSM deterrmines WHAT changes to SDA/SCL occur. 
    Phase counter determines WHICH part of the timing interval is active.
    */

  // FSM OUTLINE
  always @(posedge clk) begin
    command_finished = 1'b0;
    // Reset Logic
    if (rst) begin
      /* Reset:
      Set state to IDLE
      Clear/Initialize internal registers
      Clear active transaction & response status
      Release SDA & SCL lines
      */
      state <= ST_IDLE;
      clk_count <= 0;
      timeout_count <= 0;

      tx_shift_reg <= 8'd0;
      rx_shift_reg <= 8'd0;
      bit_index <= 3'd0;
      rsp_valid <= 1'b0;
      bus_active <= 1'b0;

      cmd_ready <= 1'b0;  // Do not accept commands during reset (ST_IDLE asserts when controller is allowed to accept command)
      rsp_data <= 8'd0;
      rsp_nack <= 1'b0;
      rsp_error <= ERR_NONE;

      sda_drive_low <= 1'b0;
      scl_drive_low <= 1'b0;
    end else begin
      case (state)
        ST_IDLE: begin
          if (command_accept) begin
            // Clear the previous result when a new command is accepted.
            rsp_valid <= 1'b0;
            rsp_data <= 8'd0;
            rsp_nack <= 1'b0;
            rsp_error <= ERR_NONE;
            // Start timing from zero
            clk_count <= 10'd0;

            opcode_latched <= cmd_opcode;   // Capture opcode
            read_nack_latched <= cmd_nack;  // Capture NACK choice
            cmd_ready <= 1'b0;              // Flag NOT ready to accept commands
            case (cmd_opcode)               // Decide action based on command opcode
              CMD_START: begin
                state <= ST_START;
                phase <= PH_START_PREPARE; // First phase of this operation
              end
              CMD_STOP: begin
                if (bus_active) begin
                  state <= ST_STOP;
                  phase <= PH_SETUP;  // First phase of this operation
                end else begin
                  rsp_error <= ERR_COMMAND;
                  command_finished = 1'b1;
                  state <= ST_ERROR;
                end
              end
              CMD_WRITE: begin
                if (bus_active) begin
                  state <= ST_WRITE;
                  phase <= PH_SETUP;  // First phase of this operation
                  bit_index <= 3'd7;        // Start with the MSB
                  tx_shift_reg <= cmd_data; // Load the data to be sent
                end else begin
                  rsp_error <= ERR_COMMAND;
                  command_finished = 1'b1;
                  state <= ST_ERROR;
                end
              end
              CMD_READ: begin
                if (bus_active) begin
                  state <= ST_READ;
                  phase <= PH_SETUP;  // First phase of this operation
                  bit_index <= 3'd7;    // Start with the MSB
                  rx_shift_reg <= 8'd0; // Clear the receive register
                end else begin
                  rsp_error <= ERR_COMMAND;
                  command_finished = 1'b1;
                  state <= ST_ERROR;
                end
              end
              default: begin
                rsp_error <= ERR_COMMAND;
                command_finished = 1'b1;
                state <= ST_ERROR;
              end
            endcase
          end else begin
            cmd_ready <= 1'b1;  // Flag ready to accept command
          end
        end
        ST_START: begin
          cmd_ready <= 1'b0;
          case (phase)
            PH_START_PREPARE: begin
              sda_drive_low <= 1'b0;
              if (!bus_active) begin
                // Initial START: release both pins and observe bus free.
                scl_drive_low <= 1'b0;
                if (sda_sync && scl_sync) begin
                  if (clk_count >= START_SETUP_CYCLES - 1) begin
                    clk_count <= 10'd0;
                    phase <= PH_HIGH;
                  end else begin
                    clk_count <= clk_count + 1'b1;
                  end
                end else begin
                  clk_count <= 10'd0; // Require a continuous HIGH interval.
                end
              end else begin
                // Repeated START: SDA must rise while SCL is still LOW.
                // Raising SDA while SCL is HIGH would generate STOP.
                scl_drive_low <= 1'b1;
                if (!scl_sync && sda_sync) begin
                  if (clk_count >= SCL_LOW_CYCLES - 1) begin
                    clk_count <= 10'd0;
                    phase <= PH_WAIT_HIGH;
                  end else begin
                    clk_count <= clk_count + 1'b1;
                  end
                end else begin
                  clk_count <= 10'd0;
                end
              end
            end
            PH_WAIT_HIGH: begin
              // Repeated START: release SCL; a slave may delay its rise.
              sda_drive_low <= 1'b0;
              scl_drive_low <= 1'b0;
              clk_count <= 10'd0;
              if (scl_sync && sda_sync) begin
                phase <= PH_HIGH;
              end
            end
            PH_HIGH: begin
              // Ensure: Both pins are released
              sda_drive_low <= 1'b0;
              scl_drive_low <= 1'b0;
              // Bus Error Check
              if (!scl_sync || !sda_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
                // Count START setup AFTER SCL is observed HIGH
              end else if (clk_count >= START_SETUP_CYCLES - 1) begin
                sda_drive_low <= 1'b1;  // SDA falls with SCL still HIGH: START.
                bus_active <= 1'b1;     // Mark bus active only once START is generated.
                clk_count <= 10'd0;
                phase <= PH_START_HOLD; // Wait START_HOLD_CYCLES before letting SCL fall
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_START_HOLD: begin
              // Ensure: SDA LOW and SCL released/HIGH after the START edge
              sda_drive_low <= 1'b1;
              scl_drive_low <= 1'b0;
              // Bus Error Check
              if (!scl_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
                // Wait HOLD cycles before letting SCL fall
              end else if (clk_count >= START_HOLD_CYCLES - 1) begin
                scl_drive_low <= 1'b1;
                clk_count <= 10'd0;
                phase <= PH_LOW;
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_LOW: begin
              // Ensure: Both lines LOW
              sda_drive_low <= 1'b1;
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                // Finish a full LOW interval before handing off the command
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  // Set Response Parameters
                  rsp_data <= 8'd0;
                  rsp_nack <= 1'b0;
                  rsp_error <= ERR_NONE;
                  rsp_valid <= 1'b1;      // Result is held and valid
                  command_finished = 1'b1;
                  state <= ST_RESPONSE;   // Confirm START finished to sequencer
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0; // Wait for the observed SCL fall first
              end
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              clk_count <= 10'd0;
              command_finished = 1'b1;
              state <= ST_ERROR;
            end
          endcase
        end
        ST_WRITE: begin
          cmd_ready <= 1'b0;
          case (phase)
            PH_SETUP: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                sda_drive_low <= ~tx_shift_reg[bit_index];
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  scl_drive_low <= 1'b0;
                  phase <= PH_WAIT_HIGH;
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            PH_WAIT_HIGH: begin
              scl_drive_low <= 1'b0;
              clk_count <= 10'd0;
              if (scl_sync) begin
                phase <= PH_HIGH;
              end
            end
            PH_HIGH: begin
              scl_drive_low <= 1'b0;
              if (!scl_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
              end else if (clk_count >= SCL_HIGH_CYCLES - 1) begin
                scl_drive_low <= 1'b1;
                clk_count <= 10'd0;
                phase <= PH_LOW;
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_LOW: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  phase <= PH_SETUP;
                  // Finished ALL bits
                  if (bit_index == 3'd0) begin
                    state <= ST_SLAVE_ACK;
                  // NOT finished
                  end else begin
                    bit_index <= bit_index - 1'b1;
                  end
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              clk_count <= 10'd0;
              command_finished = 1'b1;
              state <= ST_ERROR;
            end
          endcase
        end
        ST_SLAVE_ACK: begin
          cmd_ready <= 1'b0;
          sda_drive_low <= 1'b0;  // Release SDA for peripheral control
          case (phase)
            PH_SETUP: begin
              // Ensure: SCL LOW
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  scl_drive_low <= 1'b0;  // Release SCL
                  phase <= PH_WAIT_HIGH;  // Confirm SCL HIGH
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            PH_WAIT_HIGH: begin
              // Ensure: SCL HIGH
              scl_drive_low <= 1'b0;
              clk_count <= 10'd0;
              if (scl_sync) begin
                phase <= PH_HIGH;
              end
            end
            PH_HIGH: begin
              // Ensure: SCL HIGH
              scl_drive_low <= 1'b0;
              // Bus Error Check
              if (!scl_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
              // Wait HIGH cycles
              end else if (clk_count >= SCL_HIGH_CYCLES - 1) begin
                rsp_nack <= sda_sync;   // Read NACK from peripheral
                scl_drive_low <= 1'b1;
                clk_count <= 10'd0;
                phase <= PH_LOW;
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_LOW: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  rsp_data <= 8'd0;
                  rsp_error <= ERR_NONE;
                  rsp_valid <= 1'b1;
                  command_finished = 1'b1;
                  state <= ST_RESPONSE;
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              clk_count <= 10'd0;
              command_finished = 1'b1;
              state <= ST_ERROR;
            end
          endcase
        end
        ST_READ: begin
          cmd_ready <= 1'b0;
          sda_drive_low <= 1'b0;  // Release SDA for slave transaction
          case (phase)
            PH_SETUP: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  scl_drive_low <= 1'b0;
                  phase <= PH_WAIT_HIGH;
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            PH_WAIT_HIGH: begin
              scl_drive_low <= 1'b0;
              clk_count <= 10'd0;
              if (scl_sync) begin
                phase <= PH_HIGH;
              end
            end
            PH_HIGH: begin
              scl_drive_low <= 1'b0;
              if (!scl_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
              end else if (clk_count >= SCL_HIGH_CYCLES - 1) begin
                rx_shift_reg[bit_index] <= sda_sync;
                scl_drive_low <= 1'b1;
                clk_count <= 10'd0;
                phase <= PH_LOW;
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_LOW: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  phase <= PH_SETUP;
                  if (bit_index == 3'd0) begin
                    state <= ST_MASTER_ACK;
                  end else begin
                    bit_index <= bit_index - 1'b1;
                  end
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              clk_count <= 10'd0;
              command_finished = 1'b1;
              state <= ST_ERROR;
            end
          endcase
        end
        ST_MASTER_ACK: begin
          cmd_ready <= 1'b0;
          case (phase)
            PH_SETUP: begin
              scl_drive_low <= 1'b1;  // Ensure: SCL LOW
              if (!scl_sync) begin
                sda_drive_low <= !read_nack_latched;  // Write acknowledgement to SDA
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  scl_drive_low <= 1'b0;
                  phase <= PH_WAIT_HIGH;
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            PH_WAIT_HIGH: begin
              sda_drive_low <= !read_nack_latched;
              scl_drive_low <= 1'b0;
              clk_count <= 10'd0;
              if (scl_sync) begin
                phase <= PH_HIGH;
              end
            end
            PH_HIGH: begin
              sda_drive_low <= !read_nack_latched;
              scl_drive_low <= 1'b0;
              if (!scl_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
              end else if (clk_count >= SCL_HIGH_CYCLES - 1) begin
                scl_drive_low <= 1'b1;
                clk_count <= 10'd0;
                phase <= PH_LOW;
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_LOW: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                sda_drive_low <= 1'b0;
                if (clk_count >= SCL_LOW_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  rsp_data <= rx_shift_reg;
                  rsp_nack <= 1'b0;
                  rsp_error <= ERR_NONE;
                  rsp_valid <= 1'b1;
                  command_finished = 1'b1;
                  state <= ST_RESPONSE;
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              clk_count <= 10'd0;
              command_finished = 1'b1;
              state <= ST_ERROR;
            end
          endcase
        end
        ST_STOP: begin
          cmd_ready <= 1'b0;
          case (phase)
            PH_SETUP: begin
              scl_drive_low <= 1'b1;
              if (!scl_sync) begin
                sda_drive_low <= 1'b1;
                if (!sda_sync) begin
                  if (clk_count >= SCL_LOW_CYCLES - 1) begin
                    clk_count <= 10'd0;
                    scl_drive_low <= 1'b0;
                    phase <= PH_WAIT_HIGH;
                  end else begin
                    clk_count <= clk_count + 1'b1;
                  end
                end else begin
                  clk_count <= 10'd0;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            PH_WAIT_HIGH: begin
              sda_drive_low <= 1'b1;
              scl_drive_low <= 1'b0;
              clk_count <= 10'd0;
              if (scl_sync) begin
                phase <= PH_HIGH;
              end
            end
            PH_HIGH: begin
              sda_drive_low <= 1'b1;
              scl_drive_low <= 1'b0;
              if (!scl_sync || sda_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
              end else if (clk_count >= STOP_SETUP_CYCLES - 1) begin
                sda_drive_low <= 1'b0;
                clk_count <= 10'd0;
                phase <= PH_STOP_RELEASE;
              end else begin
                clk_count <= clk_count + 1'b1;
              end
            end
            PH_STOP_RELEASE: begin
              scl_drive_low <= 1'b0;
              sda_drive_low <= 1'b0;
              if (!scl_sync) begin
                rsp_error <= ERR_BUS;
                clk_count <= 10'd0;
                command_finished = 1'b1;
                state <= ST_ERROR;
              end else if (sda_sync) begin
                bus_active <= 1'b0;
                if (clk_count >= START_SETUP_CYCLES - 1) begin
                  clk_count <= 10'd0;
                  rsp_data <= 8'd0;
                  rsp_nack <= 1'b0;
                  rsp_error <= ERR_NONE;
                  rsp_valid <= 1'b1;
                  command_finished = 1'b1;
                  state <= ST_RESPONSE;
                end else begin
                  clk_count <= clk_count + 1'b1;
                end
              end else begin
                clk_count <= 10'd0;
              end
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              clk_count <= 10'd0;
              command_finished = 1'b1;
              state <= ST_ERROR;
            end
          endcase
        end
        ST_RESPONSE: begin
          // Check if caller accepts response
          if (response_accept) begin
            rsp_valid <= 1'b0;
            state <= ST_IDLE;
          end
        end
        ST_ERROR: begin
          cmd_ready <= 1'b0;
          clk_count <= 10'd0;
          case (rsp_error)
            ERR_COMMAND: begin
              rsp_error <= ERR_COMMAND;
            end
            ERR_TIMEOUT, ERR_BUS: begin
              sda_drive_low <= 1'b0;
              scl_drive_low <= 1'b0;
              bus_active <= 1'b0;
            end
            default: begin
              rsp_error <= ERR_COMMAND;
              sda_drive_low <= 1'b0;
              scl_drive_low <= 1'b0;
              bus_active <= 1'b0;
            end
          endcase
          rsp_data <= 8'd0;
          rsp_nack <= 1'b0;
          rsp_valid <= 1'b1;
          command_finished = 1'b1;
          state <= ST_RESPONSE;
        end
        default: begin
          cmd_ready <= 1'b0;
          rsp_error <= ERR_BUS;
          command_finished = 1'b1;
          state <= ST_ERROR;
        end
      endcase
      // TIMEOUT LOGIC
      /*
      Notes:
      Acceptance occurs in IDLE:
      - Clear the counter there
      - then count elapsed cycles until command finishes
      - Waiting for the caller is not timed
      */
      if (command_finished || state == ST_IDLE || state == ST_RESPONSE || state == ST_ERROR) begin
        timeout_count <= 0;
      end else if (timeout_count >= TIMEOUT_CYCLES - 1) begin
        timeout_count <= 0;
        cmd_ready <= 1'b0;
        rsp_valid <= 1'b0;
        rsp_error <= ERR_TIMEOUT;
        clk_count <= 10'd0;
        state <= ST_ERROR;
      end else begin
        timeout_count <= timeout_count + 1'b1;
      end
    end
  end
endmodule