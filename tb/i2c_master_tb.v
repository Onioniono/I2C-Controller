/* *********************************************************
Name: Alexander Mendoza
Role: Sensor Integration
Creation Date: 10-27-2026

File Name: i2c_master_tb.v
Purpose: Testbench for I2C Master Module for RISCV Peripherals
Revision History:
- 10-27-2026 — Create file
- 10-09-2026 — Instantiate i2c_master; Outline Tasks
********************************************************* */

// --------------------------------------------------------------------------------
// I2C Master Testbench
/*
Purpose:
- Test each individual command of i2c_master.v
Notes:
- tri1 — Models a weak pull-up
*/
// --------------------------------------------------------------------------------
// ********************** TESTBENCH **********************
module i2c_master_tb;
    reg clk;
    reg rst;
    // Commands OUT this module
    reg cmd_valid;
    wire cmd_ready;
    reg [1:0] cmd_opcode;
    reg [7:0] cmd_data;
    reg cmd_nack;
    // Responses INTO this module
    wire rsp_valid;
    reg rsp_ready;
    wire [7:0] rsp_data;
    wire rsp_nack;
    wire [1:0] rsp_error;
    // Transaction Bus
    wire bus_active;
    tri1 sda;
    tril scl;

    // Instantiate
    i2c_master #(.CLK_HZ(100_000_000), .I2C_HZ(100_000)) dut (
        .clk(clk), .rst(rst),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_opcode(cmd_opcode), .cmd_data(cmd_data), .cmd_nack(cmd_nack),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready), .rsp_data(rsp_data), .rsp_nack(rsp_nack), .rsp_error(rsp_error),
        .bus_active(bus_active), .sda(sda), .scl(scl)
    );

    // Tasks
    task initialize;
        // Initialize Inputs
        cmd_valid = 1'b0;
        cmd_opcode = 2'd0;
        cmd_data = 8'd0;
        cmd_nack = 1'b0;
        rsp_ready = 1'b0;
    endtask

    task reset_device;
        // Apply Reset
        begin
            rst = 1'b1;
            repeat (5) @(negedge clk);
            rst = 1'b0;
        end
    endtask

    task wait_reponse;
        // Wait until reponse valid
        begin
            // Inspect all response parameters
            // Assert rsp_ready
            // Wait until response_accept = rsp_valid && rsp_ready
            // Deassert rsp_ready
        end
    endtask

    task send_command;
        // Send command to master and wait for accept
        input [1:0] opcode;
        input [7:0] data_byte;
        input nack;
        begin
            // Set cmd_opcode, cmd_data, cmd_nack
            // Assert cmd_valid
            // Wait until command_accept = cmd_valid && cmd_ready
            // Deassert cmd_valid after accept
        end
    endtask

    // Clock Generation (100 MHz)
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Test Sequence
    initial begin
        // Initialize signals
        // Reset DUT
        // Test START
        // Test WRITE + slave ACK
        // Test READ + master ACK/NACK
        // Test STOP
        // Check results
        $finish;
    end
    
    // Timeout
    // <insert here>

endmodule