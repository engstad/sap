`timescale 1ms / 1us

module counter8(input            enable,
                input            load,
                input            clk,
                input            reset,
                input [7:0]      in,
                output reg [7:0] out);
   always @(posedge clk) begin
      if (reset) begin
         out <= 8'b0;
      end else if (load) begin
         out <= in;
      end else if (enable) begin
         out <= out + 2;
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
module reg8(input             clk,
            input             reset,
            input             load,
            input [7:0]       in,
            output wire [7:0] out);
   wire [3:0]                 lo_out, hi_out;
   wire [3:0]                 lo_in = in[3:0];
   wire [3:0]                 hi_in = in[7:4];
   reg4 lo(.load(load), .clk(clk), .reset(reset),
           .in(lo_in), .out(lo_out));
   reg4 hi(.load(load), .clk(clk), .reset(reset),
           .in(hi_in), .out(hi_out));
   assign out = {hi_out, lo_out};
endmodule

//
// Hopefully synthesize as distributed RAM
//
module ram(input             clk,
           // Port A
           input [7:0]       pc,
           output reg [15:0] ir,
           // Port B
           input             we,
           input [7:0]       addr,
           input [7:0]       in,
           output reg [7:0]  out);

   reg [7:0]                mem[0:(1<<8)-1];

   initial $readmemh("rom.txt", mem);

   always @(posedge clk)
     ir <= { mem[pc + 1], mem[pc] };

   always @(posedge clk)
     if (we)
       mem[addr] <= in;

   always @(posedge clk)
     out <= mem[addr];

endmodule // ram

module regs(input clk,
            input en0, input[2:0] num0, output wire[7:0] out0,
            input en1, input[2:0] num1, output wire[7:0] out1,
            input wr, input[2:0] num, input wire[7:0] in);

   reg [7:0]      regs[3:0];

   integer        j;
   initial
     for (j = 0; j < 8; j++)
       regs[j] = {8{1'b0}};

   assign out0 = en0 ? regs[num0] : 8'hZZ;

   assign out1 = en1 ? regs[num1] : 8'hZZ;

   always @(posedge clk)
     if (wr)
       regs[num] <= in;
endmodule // regs

module alu(input[7:0] a, input[7:0] b, input[7:0] c, output [7:0] out, input sub, output zero, output carry);
   assign {carry, out} = sub ? a - b : a + b + c;
   assign zero = out == 0;
   //assign pos = out > 0;
endmodule

`define MVI 4'h0
`define ADI 4'h1
`define OUT 4'h2
`define HLT 4'h3
`define NOP 4'h4


module stage1(
              input wire        clk,
              input wire        reset,

              input wire [15:0] ir_reg,
              output reg [2:0]  areg_no,
              output reg [7:0]  areg,
              output reg [2:0]  breg_no,
              output reg [7:0]  breg,
              output reg [3:0]  opcode,
              output reg [7:0]  immediate,

              // Register Interface (loads)
              output reg        regs_en0,
              output reg [2:0]  regs_no0,
              input wire [7:0]  regs_out0,

              output reg        regs_en1,
              output reg [2:0]  regs_no1,
              input wire [7:0]  regs_out1,

              // Address register interface
              output wire       addr_i,
              input wire        addr_in,
              output wire [7:0] addr_reg,

              // Flag register interface
              input wire        zero_i,
              input wire        zero_in,
              output wire       zero_out,

              // Program counter interface
              output wire       pc_i,
              output wire [7:0] pc_in,
              output wire       pc_e
              );

   //
   // ========== STAGE 1 ============
   //

   reg [3:0]                    opc;
   reg [7:0]                    imm;
   reg [2:0]                    dst;
   reg [2:0]                    src;

   always @*
     begin
        opc <= `HLT;
        imm <= 0;
        dst <= 0;
        src <= 0;

        if (ir_reg[15:11] == 5'b01011) begin
           opc <= `MVI;
           dst <= ir_reg[10:8];
           imm <= ir_reg[7:0];
        end else if (ir_reg[15:11] == 5'b10010) begin
           opc <= `ADI;
           dst <= ir_reg[9:7];
           src <= ir_reg[6:4];
           imm <= {4'b0000, ir_reg[3:0]};
        end else if (ir_reg[15:13] == 3'b111) begin
           if (ir_reg[11:8] == 4'b1110) begin
              opc <= `OUT;
              src <= ir_reg[2:0];
           end else if (ir_reg[11:8] == 4'b0000) begin
              opc <= `NOP;
           end
        end
     end

   // Setting addr_i to true will cause mem_reg <= mem[addr_reg].
   assign addr_i = 1'b0;
   assign addr_reg = 8'b00000000;

   // The A Register (ALU input)
   wire       areg_i;
   reg [7:0]  areg_in;
   reg8 areg8(.clk(clk), .reset(reset),
              .load(areg_i), .in(areg_in), .out(areg));

   // The B register (ALU input)
   wire       breg_i;
   reg [7:0]  breg_in;
   reg8 breg8(.clk(clk), .reset(reset),
              .load(breg_i), .in(breg_in), .out(breg));

   assign areg_i = (opc == `MVI || opc == `ADI || opc == `OUT);
   assign breg_i = (opc == `ADI);

   always @(*)
     if (areg_i) begin
        regs_en0 <= 1'b1;
        regs_no0 <= dst;
        areg_in <= regs_out0;
     end else begin
        regs_en0 <= 1'b0;
        regs_no0 <= 0;
        areg_in <= 0;
     end

   always @(*)
     if (breg_i) begin
        regs_en1 <= 1'b1;
        regs_no1 <= src;
        breg_in <= regs_out1;
     end else begin
        regs_en1 <= 1'b0;
        regs_no1 <= 0;
        breg_in <= 0;
     end

   // Flags
   reg1 zeroreg(.clk(clk), .reset(reset),
                .load(zero_i), .in(zero_in), .out(zero_out));

   // Jumps
   assign pc_i = 1'b0;
   assign pc_in = pc_i ? imm : 0;
   assign pc_e = !pc_i && (opc != `HLT);


   always @(posedge clk)
     if (areg_i)
       areg_no <= dst;
     else
       areg_no <= 3'b000;

   always @(posedge clk)
     if (breg_i)
       breg_no <= src;
     else
       breg_no <= 0;

   always @(posedge clk)
     opcode <= opc;

   always @(posedge clk)
     immediate <= imm;

endmodule // stage1


module sap(input wire clk, input wire reset, output reg[7:0] out);

   // Program Counter implemented as a 4-bit counter
   wire pc_e;
   wire pc_i;
   wire [7:0] pc_in; // If pc_i is true, pc_in is loaded as the new value.
   wire [7:0] pc_reg;
   counter8 pc(.enable(pc_e), .load(pc_i), .clk(clk), .reset(reset),
               .in(pc_in), .out(pc_reg));

   // Instruction register
   wire [15:0] ir_reg;

   // Address register
   wire       addr_i;
   wire [7:0] addr_in; // If addr_i is true, addr_in is the new value next clock.
   wire [7:0] addr_reg;

   reg8 ar(.load(addr_i), .clk(clk), .reset(reset), .in(addr_in), .out(addr_reg));

   // Memory register
   reg        mem_i;    // If enabled: mem[AR] <= mem_in
   reg [7:0]  mem_in;
   wire [7:0] mem_reg;  // Always:     mem_reg <= mem[AR]
   ram ram(.clk(clk),
           .pc(pc_reg),
           .ir(ir_reg),
           .addr(addr_reg),
           .we(mem_i),
           .in(mem_in),
           .out(mem_reg)
           );

   // Register File
   wire       regs_en0;
   wire [2:0] regs_no0;
   wire [7:0] regs_out0;

   wire       regs_en1;
   wire [2:0] regs_no1;
   wire [7:0] regs_out1;

   reg        regs_wr;
   reg [2:0]  regs_no;
   reg [7:0]  regs_in;

   regs registers(.clk(clk),
                  .en0(regs_en0), .num0(regs_no0), .out0(regs_out0),
                  .en1(regs_en1), .num1(regs_no1), .out1(regs_out1),
                  .wr(regs_wr), .num(regs_no), .in(regs_in));


   wire [2:0] areg_no;
   wire [7:0] areg;
   wire [2:0] breg_no;
   wire [7:0] breg;
   wire [3:0] opcode;
   wire [7:0] immediate;
   wire       zero_i;
   wire       zero_in;
   wire       zero_out;

   stage1 st1(.clk(clk),
              .reset(reset),
              //
              .ir_reg(ir_reg),
              //
              .areg_no(areg_no),
              .areg(areg),
              .breg_no(breg_no),
              .breg(breg),
              .opcode(opcode),
              .immediate(immediate),
              // register bak
              .regs_en0(regs_en0),
              .regs_no0(regs_no0),
              .regs_out0(regs_out0),
              .regs_en1(regs_en1),
              .regs_no1(regs_no1),
              .regs_out1(regs_out1),
              // memory address register
              .addr_i(addr_i),
              .addr_in(addr_in),
              .addr_reg(addr_reg),
              // not here
              .zero_i(zero_i),
              .zero_in(zero_in),
              .zero_out(zero_out),
              .pc_i(pc_i),
              .pc_in(pc_in),
              .pc_e(pc_e)
              );

   // ========= STAGE 2 ========

   // - If ALU, then do the ALU and store to register file.
   // - If store operation, then reg => mem.
   // - If load operation, then mem => reg.

   // The ALU unit
   wire       sub;
   wire [7:0] alu_y;
   reg [7:0]  alu_a;
   reg [7:0]  alu_b;
   reg [7:0]  alu_c;

   alu alu(.a(alu_a), .b(alu_b), .c(alu_c),
           .out(alu_y), .sub(sub),
           .zero(zero_in), .carry());

   reg [7:0]  alu_out;
   always @(posedge clk)
     alu_out <= alu_y;

   reg [2:0]  alu_out_no;
   always @(posedge clk)
     alu_out_no <= areg_no;

   always @*
     begin
        alu_a <= 0;
        alu_b <= 0;
        alu_c <= 0;
        if (opcode == `MVI) begin
           alu_a <= areg_no == alu_out_no ? alu_y : areg;
           alu_b <= 0;
           alu_c <= immediate;
        end else if (opcode == `ADI) begin
           alu_a <= areg_no == alu_out_no ? alu_y : areg;
           alu_b <= breg_no == alu_out_no ? alu_y : breg;
           alu_c <= immediate;
        end
     end

   assign zero_i = 1'b0;
   assign sub = 1'b0;

   always @(*)
     begin
        mem_i <= 1'b0;
        mem_in <= 0;

        regs_wr <= 1'b0;
        regs_no <= 0;
        regs_in <= 0;

        if (opcode == `ADI || opcode == `MVI) begin
           regs_wr <= 1'b1;
           regs_no <= areg_no;
           regs_in <= alu_y;
        end else begin
           regs_wr <= 1'b0;
           regs_no <= 0;
           regs_in <= 0;
        end
     end // always @ (*)

   always @(posedge clk)
     if (opcode == `OUT) begin
        out <= mem_reg;
     end

endmodule

`ifndef TEST
module top(input wire 	     sys_clk_i,
	   output wire [3:0] led,
           output wire [3:0] led_g);

   wire                      clk0_unused, clk1_unused, clk2_unused, clk3_unused, clk4_unused;
   wire                      clk_feedback, clk_locked;

   wire                      sys_clk;
   wire                      clk_feedback_buffered;

   wire                      clk;

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

   reg [16:0]                low_speed_ctr = 0;
   always @(posedge clk)
     low_speed_ctr <= low_speed_ctr + 1;

   wire [7:0]                out;

   sap sap(.clk(low_speed_ctr[16]), .reset(1'b0), .out(out));

   assign led[3:0] = out[7:4];

   reg [7:0]                 ctr = 0;
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
      #2 reset = 1'b0;
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
      $display("     PC   IR | OP A# B# Im | A   B   O");

 `else
      $display("       A   B   O");
      $monitor("%04d) %h  %h  %h ", $time, DUT.areg, DUT.breg, out);
      //$monitor("%h", out);
 `endif
      #1500 $finish;
   end // initial begin

   always @(posedge clk)
      $strobe("%3d) %2h %4h |  %h %2h %2h %2h | %2h %2h %2h %2h | %2h (%d) %2h (%d)  %h",
               $time, DUT.pc_reg, DUT.ir_reg,
               DUT.st1.opcode, DUT.st1.dst, DUT.st1.src, DUT.st1.immediate,
               DUT.registers.regs[0],
               DUT.registers.regs[1],
               DUT.registers.regs[2],
               DUT.registers.regs[3],
               DUT.areg, DUT.areg_no,
               DUT.breg, DUT.breg_no,
               out);


endmodule // tb

`endif
