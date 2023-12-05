`timescale 1ns / 1ps
module lab11_tb();
    reg clk = 0;
    reg rst;
    reg [2:0] adr1_r;
    reg [2:0] adr2_r;
    reg [2:0] adr_ram;
    wire [7:0] result;
    wire [3:0] st_out;
    Multiplier_top uut(clk, rst, adr1_r, adr2_r, adr_ram, result, st_out);
    
    initial begin
        forever begin
            #10 clk = ~clk;
        end
    end
    
    initial begin
        #10 rst = 1;
        #10 rst = 0; adr_ram = 3'b0;
        #10 adr1_r = 3'd010; //value at 2 is: 6
        #10 adr2_r = 3'd110; //value at 6 is: 13
        #200;
    end
        
            
    
endmodule

