/* *********************************************************
Name: Alexander Mendoza
Role: Sensor Integration
Creation Date: 10-27-2026

File Name: i2c_master_tb.v
Purpose: Testbench for I2C Master Module for RISCV Peripherals
Revision History:
- 10-27-2026 — Create file
- 11-09-2026 — Instantiate i2c_master; Outline Tasks
- 11-12-2026 — Add: tasks; basic test logic; timeout
********************************************************* */

`timescale 1ns/1ps

// --------------------------------------------------------------------------------
// I2C Master Testbench
/*
Purpose:
- Generic self-checking test for i2c_master
Notes:
- Vivado: add this file to Simulation Sources; add the master to Design Sources
    - Select i2c_master_tb as simulation top, then Run Behavioral Simulation / Run All
*/
// --------------------------------------------------------------------------------
// ******************************************************* TESTBENCH *******************************************************
module i2c_master_tb;
    localparam integer CLK_HZ = 100_000_000;
    localparam [1:0] CMD_START=0, CMD_STOP=1, CMD_WRITE=2, CMD_READ=3;
    reg clk;
    reg rst;
    // Commands OUT of this module
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
    // Each simulated device can only pull LOW or release the pulled-up bus
    wire bus_active;
    tri1 sda;
    tri1 scl;
    reg slave_sda_low;
    assign sda = slave_sda_low ? 1'b0 : 1'bz;
    integer failures=0, checks=0, responses=0;
    integer starts=0, stops=0, rising_edges=0;
    integer before_start, before_stop, before_edges;

    // Instantiate the master with its normal 100 MHz / 1 ms settings
    i2c_master #(.CLK_HZ(CLK_HZ), .I2C_HZ(100_000), .TIMEOUT_MS(1)) dut (
        .clk(clk), .rst(rst),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_data(cmd_data), .cmd_nack(cmd_nack),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready), .rsp_data(rsp_data),
        .rsp_nack(rsp_nack), .rsp_error(rsp_error), .bus_active(bus_active),
        .sda(sda), .scl(scl)
    );

    // Tasks for normal command stimulus and basic result checks
    task expect_true;
        input condition;
        input [8*100-1:0] description;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) begin
                failures = failures + 1;
                $display("FAIL @ %0t: %0s", $time, description);
            end
        end
    endtask

    task initialize;
        begin
            rst=1; cmd_valid=0; cmd_opcode=0; cmd_data=0; cmd_nack=0;
            rsp_ready=0; slave_sda_low=0;
        end
    endtask

    task reset_device;
        begin
            @(negedge clk);
            rst=1; cmd_valid=0; rsp_ready=0;
            slave_sda_low=0;
            repeat (5) @(negedge clk);
            expect_true(!rsp_valid && !cmd_ready && !bus_active, "reset clears status");
            expect_true(sda === 1'b1 && scl === 1'b1, "reset releases both pins");
            rst=0;
            repeat (5) @(negedge clk);
            expect_true(cmd_ready, "ready after reset");
        end
    endtask

    task send_command;
        input [1:0] opcode;
        input [7:0] data_byte;
        input nack;
        begin
            @(negedge clk);
            cmd_opcode=opcode; cmd_data=data_byte; cmd_nack=nack; cmd_valid=1;
            @(posedge clk);
            while (cmd_ready !== 1'b1) begin
                @(posedge clk);
            end
            @(negedge clk);
            cmd_valid=0;
        end
    endtask

    task wait_response;
        input [7:0] expected_data;
        begin
            while (rsp_valid !== 1'b1) @(negedge clk);
            expect_true(rsp_data === expected_data, "response data");
            expect_true(rsp_nack === 1'b0, "normal command has no slave NACK");
            expect_true(rsp_error === 2'd0, "command completed without error");
            @(negedge clk); rsp_ready=1;
            @(negedge clk); rsp_ready=0;
            repeat (4) @(negedge clk);
            expect_true(!rsp_valid && cmd_ready, "response consumed; ready for next command");
        end
    endtask

    // Receive eight physical bits, then acknowledge the master on clock nine
    task slave_receive;
        input [7:0] expected;
        reg [7:0] observed;
        integer bit_number;
        begin
            slave_sda_low=0;
            for (bit_number=7; bit_number>=0; bit_number=bit_number-1) begin
                @(posedge scl); observed[bit_number]=sda;
            end
            expect_true(observed === expected, "wire-level WRITE byte, MSB first");
            @(negedge scl); #100; slave_sda_low=1'b1;
            @(posedge scl);
            expect_true(sda === 1'b0, "slave drives ACK on ninth clock");
            @(negedge scl); #100; slave_sda_low=0;
        end
    endtask

    // Send a byte and inspect the master's ACK/NACK on clock nine
    task slave_transmit;
        input [7:0] value;
        input expected_nack;
        integer bit_number;
        begin
            for (bit_number=7; bit_number>=0; bit_number=bit_number-1) begin
                #100; slave_sda_low=!value[bit_number];
                @(posedge scl);
                @(negedge scl);
            end
            #100; slave_sda_low=0;
            @(posedge scl);
            expect_true(sda === expected_nack, "master drives selected ACK/NACK on clock nine");
            @(negedge scl);
        end
    endtask

    task start_transaction;
        begin
            before_start=starts; before_stop=stops;
            send_command(CMD_START,0,0);
            wait_response(0);
            expect_true(starts == before_start+1 && stops == before_stop,
                        "one START, no unintended STOP (also repeated START)");
            expect_true(bus_active && scl === 1'b0, "START leaves active bus with SCL LOW");
        end
    endtask

    task stop_transaction;
        begin
            before_stop=stops; before_start=starts;
            send_command(CMD_STOP,0,0);
            wait_response(0);
            expect_true(stops == before_stop+1 && starts == before_start,
                        "one STOP, no unintended START");
            expect_true(!bus_active && sda === 1'b1 && scl === 1'b1,
                        "STOP leaves bus released and inactive");
        end
    endtask

    task write_byte;
        input [7:0] value;
        begin
            before_edges=rising_edges;
            fork
                send_command(CMD_WRITE,value,0);
                slave_receive(value);
            join
            wait_response(0);
            expect_true(rising_edges == before_edges+9, "WRITE has eight bits plus one ACK clock");
            expect_true(bus_active, "WRITE preserves transaction");
        end
    endtask

    task read_byte;
        input [7:0] value;
        input nack;
        begin
            before_edges=rising_edges;
            fork
                send_command(CMD_READ,0,nack);
                slave_transmit(value,nack);
            join
            wait_response(value);
            expect_true(rising_edges == before_edges+9, "READ has eight bits plus one ACK clock");
        end
    endtask

    // Clock generation (100 MHz)
    initial clk=0;
    always #5 clk=~clk;

    // Protocol observations (no dependence on the DUT state encoding)
    always @(negedge sda) if (!rst && scl === 1'b1) starts=starts+1;
    always @(posedge sda) if (!rst && scl === 1'b1) stops=stops+1;
    always @(posedge scl) if (!rst) rising_edges=rising_edges+1;
    always @(posedge clk)
        if (!rst && rsp_valid && rsp_ready) responses=responses+1;

    // Basic successful commands only (the byte values have no specific meaning)
    initial begin
        initialize;
        reset_device;

        $display("TEST: START");
        start_transaction;

        $display("TEST: WRITE one byte, slave ACK");
        write_byte(8'ha5);

        $display("TEST: repeated START");
        start_transaction;

        $display("TEST: READ one byte, master ACK");
        read_byte(8'h3c,0);

        $display("TEST: READ final byte, master NACK");
        read_byte(8'h5a,1);

        $display("TEST: STOP");
        stop_transaction;

        if (failures != 0) $fatal(1,"FAIL: %0d of %0d checks failed", failures,checks);
        $display("PASS: %0d checks, %0d response handshakes",checks,responses);
        $finish;
    end

    // Simulation safeguard only: prevents an infinite run if a command gets stuck
    initial begin
        #2000000;
        $fatal(1,"global testbench watchdog expired (2 ms)");
    end

endmodule
