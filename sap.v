`timescale 1ms / 1us

module counter4(input enable,
                input            load,
                input            clk,
                input            reset,
                input [3:0]      in,
                output reg [3:0] out);
   always @(posedge clk) begin
      if (reset) begin
         out <= 4'b0;
      end else if (load) begin
         out <= in;
      end else if (enable) begin
         out <= out + 1;
      end
   end // @(posedge clk)
endmodule

module reg1(input load, input clk, input reset, input in, output reg out);
   always @(posedge clk)
     if (reset)
       out <= 1'b1;
     else if (load)
       out <= in;
endmodule // reg

// A 4-bit register
module reg4(input load, input clk, input reset, input[3:0] in,
            output reg [3:0] out);
   always @(posedge clk)
     if (reset) begin
        out <= 4'b0;
     end else if (load) begin
        out <= in;
     end
endmodule

// An 8-bit register (implemented w/2 4-bit registers)
module reg8(input load_lo, input load_hi, input clk, input reset, input[7:0] in,
            output wire [7:0] out);
   wire [3:0]                 lo_out, hi_out;
   wire [3:0]                 lo_in = in[3:0];
   wire [3:0]                 hi_in = in[7:4];
   reg4 lo(.load(load_lo), .clk(clk), .reset(reset),
           .in(lo_in), .out(lo_out));
   reg4 hi(.load(load_hi), .clk(clk), .reset(reset),
           .in(hi_in), .out(hi_out));
   assign out = {hi_out, lo_out};
endmodule

`define NOP 4'h0
`define LDA 4'h1
`define ADD 4'h2
`define SUB 4'h3
`define STA 4'h4
`define LDI 4'h5
`define JMP 4'h6
`define JC  4'h7
`define JZ  4'h8
`define LDB 4'h9
`define STB 4'hA
`define ADI 4'hB
`define SBI 4'hC
`define JNZ 4'hD
`define OUT 4'hE
`define HLT 4'hF

//
// Hopefully synthesize as distributed RAM
//
module ram(input        clk,
           input        write_enable,
           input [3:0]  addr,
           input [7:0]  data_in,
           output [7:0] data_out);

   reg [7:0]                mem[0:(1<<4)-1];

   initial $readmemh("rom.txt", mem);

   always @(posedge clk)
     if (write_enable)
       mem[addr] <= data_in;

   assign data_out = mem[addr];
endmodule // ram

// The micro-op ROM
module ctrl(input [3:0]       opcode,
            input             zero,
            input             carry,
            input             clk,
            input             reset,
            output reg [15:0] out);

   localparam PC_I = 1 << 0; // program counter input (from bus) enable
   localparam PC_O = 1 << 1; // program counter output (to bus) enable
   localparam PC_E = 1 << 2; // program counter enable (increment)
   localparam MAR_I = 1 << 3; // memory address register input (from bus)
   localparam MEM_O = 1 << 4; // output ROM memory to bus
   localparam IR_I = 1 << 5; // instruction register input (from bus)
   localparam IR_O = 1 << 6; // instruction register output (to bus)
   localparam AREG_L = 1 << 7; // A register load low nibble (from bus)
   localparam AREG_H = 1 << 8; // A register load high nibble (from bus)
   localparam AREG_O = 1 << 9;
   localparam BREG_I = 1 << 10; // B register load (from bus)
   localparam BREG_O = 1 << 11;
   localparam ALU_O = 1 << 12; // ALU output
   localparam OUT_I = 1 << 13; // OUT register load (from bus)
   localparam MEM_I = 1 << 14; //
   localparam SUB = 1 << 15;

   localparam AREG_I = AREG_L | AREG_H;

   reg [1:0]                  state = S0;
   reg [1:0]                  next_state;

   parameter
     S0 = 2'b00,
     S1 = 2'b01,
     S2 = 2'b10,
     S3 = 2'b11;

   always @*
     case (state)
       S0: next_state = S1;
       S1: next_state = S2;
       S2: case (opcode)
             `LDA, `STA, `LDB, `STB, `OUT: next_state = S3;
             default: next_state = S0;
           endcase
       S3: next_state = S0;
     endcase

   always @*
     case (state)
       // Send the program counter (PC) to the memory address register (MAR)
       S0: out = MAR_I | PC_O;
       // Send the memory output (at PC) to the instruction register (IR)
       S1: out = IR_I | MEM_O;
       S2: case (opcode)
             // Increment program counter
             `NOP: out =  PC_E;
             // Send the instruction register (low nibble) to the memory address register.
             `LDA: out =  MAR_I | IR_O;
             `STA: out =  MAR_I | IR_O;
             `LDB: out =  MAR_I | IR_O;
             `STB: out =  MAR_I | IR_O;
             `OUT: out =  MAR_I | IR_O;
             // Send the ALU output (A+B) into the A register, and increment program counter.
             `ADD: out =  AREG_I | ALU_O | PC_E;
             // Send the ALU output (A-B) into the A register, and increment program counter.
             `SUB: out =  AREG_I | ALU_O | SUB | PC_E;
             // Send the instruction register (low nibble) to the low bits of the A register.
             `LDI: out =  AREG_L | IR_O | PC_E;
             // Send the instruction register (low nibble) to the program counter.
             `JMP: out =  PC_I | IR_O;
             // Send the instruction register (low nibble) to the program counter if the zero flag is set.
             `JZ:  out =  zero ? (PC_I | IR_O) : PC_E;
             // Send the instruction register (low nibble) to the program counter if the zero flag is not set.
             `JNZ: out =  zero ? PC_E : (PC_I | IR_O);
             // Do nothing.
             `HLT: out =  0;
             default: out =  0;
           endcase // case (opcode)
       S3: case (opcode)
             // AREG = MEM
             `LDA: out =  AREG_I | MEM_O | PC_E;
             // MEM = AREG
             `STA: out =  MEM_I | AREG_O | PC_E;
             // BREG = MEM
             `LDB: out =  BREG_I | MEM_O | PC_E;
             // MEM = BREG
             `STB: out =  MEM_I | BREG_O | PC_E;
             // OUT = MEM
             `OUT: out =  OUT_I | MEM_O | PC_E;
             default: out =  0;
           endcase // case (opcode)
     endcase // case (state)

   always @(posedge clk or posedge reset) begin
      if (reset) begin
         state <= S0;
      end else begin
         state <= next_state;
      end
   end

endmodule

module alu(input[7:0] a, input[7:0] b, output [7:0] out, input sub, output zero, output carry);
   assign {carry, out} = sub ? a - b : a + b;
   assign zero = out == 0;
   //assign pos = out > 0;
endmodule

module buf8(input wire[7:0] x, input wire enable, output wire[7:0] y);
   assign y = enable ? x : 8'hZZ;
endmodule

module sap(input wire clk, input wire reset, output wire[7:0] out);
   //localparam HI = 1'b1;
   //localparam LO = 1'b0;

   // Control signals
   wire mem_i;
   wire sub, out_i, alu_o, breg_o, breg_i, areg_o, areg_h, areg_l, ir_o, ir_i, mem_o, mar_i, pc_e, pc_o, pc_i;

   // The bus
   wire [7:0] bus;

   // Program Counter implemented as a 4-bit counter
   wire [3:0] pc_reg;
   counter4 pc(.enable(pc_e), .load(pc_i), .clk(clk), .reset(reset),
               .in(bus[3:0]), .out(pc_reg));
   buf8 po({4'b0000, pc_reg}, pc_o, bus);

   // Memory Address Register as a 4-bit register
   wire [3:0] mar_reg;
   reg4 mar(.load(mar_i), .clk(clk), .reset(reset), .in(bus[3:0]), .out(mar_reg));

   // The program ROM is 4-bit addressable 8-bit memory.
   wire [7:0] mem_reg;
   ram ram(.clk(clk),
           .write_enable(mem_i),
           .addr(mar_reg),
           .data_in(bus),
           .data_out(mem_reg)
           );
   buf8 memo(mem_reg, mem_o, bus);


   // The Instruction Register
   wire [7:0] ireg_reg;
   reg8 ireg(.load_lo(ir_i), .load_hi(ir_i), .clk(clk), .reset(reset), .in(bus),
             .out(ireg_reg));
   buf8 irego({4'b0000, ireg_reg[3:0]}, ir_o, bus);

   // The A Register (ALU input)
   wire [7:0] areg_reg;
   reg8 areg(.load_lo(areg_l), .load_hi(areg_h), .clk(clk), .reset(reset), .in(bus),
             .out(areg_reg));
   buf8 arego(areg_reg, areg_o, bus);

   // The B register (ALU input)
   wire [7:0] breg_reg;
   reg8 breg(.load_lo(breg_i), .load_hi(breg_i), .clk(clk), .reset(reset), .in(bus),
             .out(breg_reg));
   buf8 brego(breg_reg, breg_o, bus);

   // The ALU unit
   wire [7:0] alu_reg;
   wire       zero_flag, carry_flag;
   alu alu(.a(areg_reg), .b(breg_reg),
           .out(alu_reg), .sub(sub),
           .zero(zero_flag), .carry(carry_flag));
   buf8 aluo(alu_reg, alu_o, bus);

   // Flags
   wire       zero;
   wire       carry;
   reg1 zeroreg(.load(alu_o), .clk(clk), .reset(reset), .in(zero_flag), .out(zero));
   reg1 carryreg(.load(alu_o), .clk(clk), .reset(reset), .in(carry_flag), .out(carry));

   // The O (output) register
   reg8 oreg(.load_lo(out_i), .load_hi(out_i), .clk(clk), .reset(reset), .in(bus),
             .out(out));

   // And finally the control unit
   ctrl ctrl(.clk(clk), .reset(reset), .opcode(ireg_reg[7:4]), .zero(zero), .carry(carry),
             .out({sub, mem_i, out_i, alu_o, breg_o, breg_i, areg_o, areg_h, areg_l, ir_o, ir_i, mem_o, mar_i, pc_e, pc_o, pc_i}));
endmodule

`ifndef TEST
module top(input wire 	     sys_clk_i,
	   output wire [3:0] led,
           output wire [3:0] led_g);

   wire                    clk0_unused, clk1_unused, clk2_unused, clk3_unused, clk4_unused;
   wire                    clk_feedback, clk_locked;

   wire                    sys_clk;
   wire                    clk_feedback_buffered;

   wire                    clk;

   PLLE2_BASE #(.BANDWIDTH("OPTIMIZED"),
                .CLKFBOUT_MULT(12), // Multiply value for all outputs (2-64)
                .CLKFBOUT_PHASE("0.0"),
                .CLKIN1_PERIOD("10.0"), // Input clock period [ns]
                .CLKOUT0_DIVIDE(6), // 200 MHz
                .CLKOUT1_DIVIDE(6), // 200 MHz
                .CLKOUT2_DIVIDE(3), // 100 MHz
                .CLKOUT3_DIVIDE(25), // 48 MHz
                .CLKOUT4_DIVIDE(50), // 24 MHz
                .CLKOUT5_DIVIDE(100), // 12 MHz
                .CLKOUT0_DUTY_CYCLE("0.5"),
                .CLKOUT1_DUTY_CYCLE("0.5"),
                .CLKOUT2_DUTY_CYCLE("0.5"),
                .CLKOUT3_DUTY_CYCLE("0.5"),
                .CLKOUT4_DUTY_CYCLE("0.5"),
                .CLKOUT5_DUTY_CYCLE("0.5"),
                .CLKOUT0_PHASE("0.0"),
                .CLKOUT1_PHASE("90.0"),
                .CLKOUT2_PHASE("0.0"),
                .CLKOUT3_PHASE("0.0"),
                .CLKOUT4_PHASE("0.0"),
                .CLKOUT5_PHASE("0.0"),
                .DIVCLK_DIVIDE(1), // Master division value (1-56)
                .REF_JITTER1("0.0"),  // Jitter (0.000-0.999)
                .STARTUP_WAIT("TRUE")
                )
   genclock(.CLKOUT0(clk0_unused),
            .CLKOUT1(clk1_unused), // ddr clock
            .CLKOUT2(clk2_unused),
            .CLKOUT3(clk3_unused),
            .CLKOUT4(clk4_unused),
            .CLKOUT5(clk),
            .CLKFBOUT(clk_feedback),
            .LOCKED(),
            .CLKIN1(sys_clk),
            .PWRDWN(1'b0),
            .RST(1'b0),
            .CLKFBIN(clk_feedback_buffered)
            );

   BUFH feedback_buffer(.I(clk_feedback), .O(clk_feedback_buffered));
   IBUF sysclk_buffer(.I(sys_clk_i), .O(sys_clk));

   reg [16:0]              low_speed_ctr = 0;
   always @(posedge clk)
     low_speed_ctr <= low_speed_ctr + 1;

   wire [7:0]              out;

   sap sap(.clk(low_speed_ctr[16]), .reset(1'b0), .out(out));

   assign led[3:0] = out[7:4];

   reg [7:0]               ctr = 0;
   always @(posedge clk)
     ctr <= ctr + 1;

   assign led_g[3:0] = (ctr == 0) ? out[3:0] : 4'b0000;

endmodule


`else


module tb();
   reg clk, reset;
   wire [7:0] out;

   sap DUT (.clk(clk), .reset(reset), .out(out));

   initial begin
      clk = 1'b1;
      reset = 1'b1;
      #10 reset = 1'b0;
   end

   initial begin
      forever #1 clk = ~clk;
   end

   initial begin
      $dumpfile("sap.vcd");
      $dumpvars;
   end

   initial begin
      `ifdef DETAILED
      $display("     PC  S  IR MAR MEM   A   B   O");
      $monitor("%3d) %2h %2h  %h  %2h  %2h  %h  %h  %h  (%h)", $time,
               DUT.pc.out, DUT.ctrl.state,
               DUT.ireg.out, DUT.mar.out, DUT.mem_reg,
               DUT.areg.out, DUT.breg.out, out, DUT.bus);
      `else
      //$display("       A   B   O");
      //$monitor("%04d) %h  %h  %h ", $time, DUT.areg.out, DUT.breg.out, out);
      $monitor("%h", out);
      `endif
      #1500 $finish;
   end
endmodule // tb
`endif
