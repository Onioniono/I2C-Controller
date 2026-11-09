/* *********************************************************
Name: Alexander Mendoza
Role: Sensor Integration
Creation Date: 10-27-2026

File Name: i2c_master_tb.v
Purpose: Testbench for I2C Master Module for RISCV Peripherals
Revision History:
- 10-27-2026: Create file
********************************************************* */

module i2c_master_tb;
    reg clk;
    reg rst;

    task reset_device;
    endtask

    task wait_reponse;
    endtask

    task send_command;
    endtask

    initial begin
        $finish;
    end
endmodule