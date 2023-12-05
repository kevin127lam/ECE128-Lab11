`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
module ram(i_clk, i_rst, i_write_en, i_addr, i_write_data, o_read_data);
    input i_clk, i_rst, i_write_en;
    input [2:0] i_addr;
    input [7:0] i_write_data;
    output reg [7:0] o_read_data;
    reg [7:0] mem [0:7];
    
    always @ (posedge i_clk) begin
        if(i_rst) begin
            mem[0]<= 8'b0;
            mem[1]<= 8'b0;
            mem[2]<= 8'b0;
            mem[3]<= 8'b0;
            mem[4]<= 8'b0;
            mem[5]<= 8'b0;
            mem[6]<= 8'b0;
            mem[7]<= 8'b0;
        end 
        else begin
            if(i_write_en)
                mem[i_addr] <= i_write_data;
            else
                o_read_data <= mem[i_addr];
            end
        end
            
endmodule

module rom(ROM_data, ROM_addr);
    output reg [3:0] ROM_data;
    input[2:0] ROM_addr;
    always @ (ROM_addr) begin
        case (ROM_addr)
         3'd0: ROM_data = 4'b0000;
         3'd1: ROM_data = 4'b1100;
         3'd2: ROM_data = 4'b0110;
         3'd3: ROM_data = 4'b0111;
         3'd4: ROM_data = 4'b1000;
         3'd5: ROM_data = 4'b0001;
         3'd6: ROM_data = 4'b1101;
         3'd7: ROM_data = 4'b1110;
    endcase
  end
endmodule

module seq_multiplier(a,b,product);

    input [3:0] a,b;
    wire [3:0]m0;
    wire [4:0]m1;
    wire [5:0]m2;
    wire [6:0]m3;
    
    wire [7:0] s1,s2,s3;
    output [7:0] product;
    
    assign m0 = {4{a[0]}} & b[3:0];
    assign m1 = {4{a[1]}} & b[3:0];
    assign m2 = {4{a[2]}} & b[3:0];
    assign m3 = {4{a[3]}} & b[3:0];
    
    
    assign s1 = m0 + (m1<<1);
    assign s2 = s1 + (m2<<2);
    assign s3 = s2 + (m3<<3);
    assign product = s3;
    
endmodule

module lab11(

    );
endmodule

module RF(A, B, SA, SB, D, DA, W, rst, clk);
	output [3:0]A; // A bus
	output [3:0]B; // B bus
	input SA; // Select A - A Address
	input SB; // Select B - B Address
	input [3:0]D; // Data input
	input DA; // Data destination address
	input W; // write enable
	input rst; // positive logic asynchronous reset
	input clk;
	
	wire [1:0]load_enable;
	wire [3:0]R00, R01;
	
	
	Decoder1to2 decoder (load_enable, DA, W);
	RegisterNbit reg00 (D,R00,load_enable[0], rst, clk); //D-in, R00-out
	RegisterNbit reg01 (D,R01,load_enable[1], rst, clk);
	Mux2to1Nbit muxA (A,R00, R01, SA);
	Mux2to1Nbit muxB (B,R00, R01,SB); 

endmodule

module RegisterNbit(D, Q,  L, R, clock);
	parameter N = 4; // number of bits
	output reg [N-1:0]Q; // registered output
	input [N-1:0]D; // data input
	input L; // load enable
	input R; // positive logic asynchronous reset
	input clock; // positive edge clock
	
	always @(posedge clock or posedge R) begin
		if(R)
			Q <= 0;
		else if(L)
			Q <= D;
		else
			Q <= Q;
	end
endmodule

module Decoder1to2(m, S, en);
	input S; // select
	input en; // enable (positive logic)
	output [1:0]m; // 32 minterms
	
	assign m[0] = ~S&en;
	assign m[1] = S&en;
	
endmodule

module Mux2to1Nbit(o, i1,i2, s);
   input [3:0] i1,i2;
   input  s;
   output reg  [3:0] o;
 
always @(s or i1 or i2)
begin
   case (s)
      1'b0 : o = i1;
      1'b1 : o = i2;
      default : o = 4'b0;
   endcase
end
endmodule

module cu ( 
input clk, reset,
  input [2:0] adr1,
  input [2:0] adr2,
  output reg w_rf,
  output reg [2:0] adr,
  output reg DA,SA,SB,
  output reg [3:0] st_out,
  output reg w_ram // [2:0]
);
  
parameter S0_idle = 0 , S1_send_adr1 = 1 , S2_send_adr2 = 2 ,S3_multiply = 3 ,S4_write_ram =
4,S5_read_ram=5 ;
reg [3:0] PS,NS ;
always@(posedge clk or posedge reset)
    begin
    if(reset)
        PS <=S0_idle;
    else
        PS <= NS ;
    end

always@(*)
    begin
    case(PS)
    
        S0_idle:begin
        NS = S1_send_adr1;
        w_rf <= 0;
        w_ram <= 1'b0;
        st_out <= S0_idle;
    end

S1_send_adr1:begin
    w_rf <= 1'b1;
    adr<= adr1;
    DA <=1'b0;
    SA <=1'b0;
    SB <=1'b1;
    st_out <= S1_send_adr1;
    NS <= S2_send_adr2;
end

S2_send_adr2:begin
    w_rf <= 1'b1;
    adr<= adr2;
    NS <= S3_multiply;
    DA <=1'b1;
    SA <=1'b0;
    SB <=1'b1;
    st_out <= S2_send_adr2;

end

S3_multiply: begin
    NS <= S4_write_ram;
    st_out <= S3_multiply;
    w_ram<= 1'b1;
end

S4_write_ram: begin
    st_out <= S4_write_ram;
    NS <= S5_read_ram;
end

S5_read_ram: begin
    w_ram<= 1'b0;
    st_out <= S5_read_ram;
    if(!reset) begin
        NS <= S5_read_ram;
    end
    else begin
        NS <= S0_idle;
    end
    end
    endcase
end
endmodule

module Multiplier_top(clk,rst,adr1_r,adr2_r,adr_ram,result,st_out);
output [7:0]result;
output [3:0] st_out; //could be 1 bit
input clk,rst;
input [2:0]adr1_r,adr2_r,adr_ram;
wire write, DA, SA, SB, w_r;
wire [2:0] rom_adr;
wire [3:0] rom_data, data1, data2;
wire [7:0] product;


rom uut1(.ROM_data(rom_data), .ROM_addr(rom_adr));
//  output reg [3:0] ROM_data;
//    input[2:0] ROM_addr;

cu uut4(.clk(clk), .reset(rst), .adr1(adr1_r), .adr2(adr2_r), .w_rf(write)
,.adr(rom_adr), .DA(DA), .SA(SA), .SB(SB), .st_out(st_out), .w_ram(w_r));
//input clk, reset,
//  input [2:0] adr1,
//  input [2:0] adr2,
//  output reg w_rf,
//  output reg [2:0] adr,
//  output reg DA,SA,SB,
//  output reg [3:0] st_out,
//  output reg w_ram // [2:0]

RF uut5(.A(data1), .B(data2), .SA(SA), .SB(SB), .D(rom_data), .DA(DA), .W(write), .rst(rst), .clk(clk)); 
//output [3:0]A; // A bus
//	output [3:0]B; // B bus
//	input SA; // Select A - A Address
//	input SB; // Select B - B Address
//	input [3:0]D; // Data input
//	input DA; // Data destination address
//	input W; // write enable
//	input rst; // positive logic asynchronous reset
//	input clk;

seq_multiplier uut2(.a(data1), .b(data2), .product(product)); //Use combinational -1 from the slide


ram uut3(.i_clk(clk), .i_rst(rst), .i_write_en(w_r), .i_addr(adr_ram), .i_write_data(product), .o_read_data(result));
//  input i_clk, i_rst, i_write_en;
//    input [2:0] i_addr;
//    input [7:0] i_write_data;
//    output reg [7:0] o_read_data;

endmodule

