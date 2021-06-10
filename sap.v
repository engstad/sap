`timescale 1ms / 1us

module counter8(input            clk,
                input            reset,
                input            enable,
                input            load,
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

// A 8-bit register
module reg8(input load, input clk, input reset, input[3:0] in,
            output reg [8-1:0] out);
   always @(posedge clk)
     if (reset) begin
        out <= 8'b0;
     end else if (load) begin
        out <= in;
     end
endmodule


//
// Hopefully synthesize as distributed RAM
//
module ram(input             clk,
           input             enable,
           // Port A
           input [7:0]       pc,
           output reg [15:0] ir,
           // Port B
           input             we,
           input [7:0]       addr,
           input [7:0]       in,
           output reg [7:0]  out);

   reg [7:0]                 mem[0:(1<<8)-1];

   initial $readmemh("rom.txt", mem);

   always @(posedge clk)
     if (enable)
       ir <= { mem[pc + 1], mem[pc] };

   always @(posedge clk)
     if (we && enable)
       mem[addr] <= in;

   always @(posedge clk)
     if (enable)
       out <= mem[addr];

endmodule // ram

module regs(input clk,
            input en0, input[2:0] num0, output wire[7:0] out0,
            input en1, input[2:0] num1, output wire[7:0] out1,
            input wr, input[2:0] num, input wire[7:0] in);

   reg [7:0]      regs[7:0];

   /*
    initial begin
    regs[0] = 0;
    regs[1] = 0;
    regs[2] = 0;
    regs[3] = 0;
    regs[4] = 0;
    regs[5] = 0;
    regs[6] = 0;
    regs[7] = 0;
   end
    */

   assign out0 = en0 ? regs[num0] : 8'hFF;
   assign out1 = en1 ? regs[num1] : 8'hFF;

   always @(posedge clk)
     if (wr)
       regs[num] <= in;

endmodule // regs

module adder(input[7:0] a, input[7:0] b, output [7:0] out, input sub, output zero, output carry);
   assign {carry, out} = sub ? a - b : a + b;
   assign zero = out == 0;
endmodule // adder

module shifter(input [7:0] a, input[2:0] sh, output [7:0] out, input[1:0] op);
   assign out = (op == 0) ? (a << sh) :
                (op == 1) ? (a >> sh) :
                (op == 2) ? ($signed(a) >>> sh) :
                ((a >> sh) | (a << (8 - sh)));
endmodule // shifter


`define OP_SHL 4'h0
`define OP_LSR 4'h1
`define OP_ASR 4'h2
`define OP_ROR 4'h3
`define OP_AND 4'h4
`define OP_XOR 4'h5
`define OP_TST 4'h6   // AND w/no results (cond flags)
`define OP_OR  4'h7
`define OP_BIC 4'h8   // D <- R & ~I
`define OP_MVN 4'h9   // D <- ~S
`define OP_ADC 4'hA   // Add w/carry
`define OP_SBC 4'hB   // Sub w/carry
`define OP_ADD 4'hC   // Also CMN: A + B => cond
`define OP_SUB 4'hD   // Also CMP: A - B => cond
`define OP_MUL 4'hE   // D <- A * B
`define OP_RSB 4'hF   // D <- I - A

module alu(input[7:0] a, input[7:0] b, output [7:0] y, input[3:0] op, input ci, output z, output co);
   wire [3:0] sh;

   assign sh  = b[3:0];

   assign {co, y} = (op == `OP_SHL) ? (a << sh) :
                    (op == `OP_LSR) ? (a >> sh) :
                    (op == `OP_ASR) ? $unsigned($signed(a) >>> sh) :
                    (op == `OP_ROR) ? ((a >> sh) | (a << (8 - sh))) :
                    (op == `OP_AND) ? (a & b) :
                    (op == `OP_XOR) ? (a ^ b) :
                    (op == `OP_TST) ? (a & b) :
                    (op == `OP_OR ) ? (a | b) :
                    (op == `OP_BIC) ? (a & ~b) :
                    (op == `OP_MVN) ? (~a) :
                    (op == `OP_ADC) ? (a + b + ci) :
                    (op == `OP_SBC) ? (a + !b + ci) :
                    (op == `OP_ADD) ? (a + b) :
                    (op == `OP_SUB) ? (a - b) :
                    (op == `OP_MUL) ? (a * b) :
                    (op == `OP_RSB) ? (b - a) : 9'h000;

   assign z = (y == 0);
endmodule

`define C_NOP  4'h0
`define C_ALU  4'h1
`define C_BRA  4'h2
`define C_OUT  4'h3
`define C_HALT 4'h4

module stage1(input wire        clk,
              input wire        reset,

              // Input
              input wire        enable,

              input wire [15:0] ir_reg,

              // Outputs
              output reg        o_dest_valid,
              output reg [2:0]  o_dest_no,

              output reg        o_src0_valid,
              output reg [2:0]  o_src0_no,
              output reg [7:0]  o_src0_reg,

              output reg        o_src1_valid,
              output reg [2:0]  o_src1_no,
              output reg [7:0]  o_src1_reg,

              output reg [3:0]  o_opc_reg,
              output reg [3:0]  o_alu_reg,
              output reg        o_alu_valid,

              output reg        o_imm_valid,
              output reg [7:0]  o_imm_reg,

              // Register Interface (loads)
              output wire       regs_en0,
              output wire [2:0] regs_no0,
              input wire [7:0]  regs_out0,

              output wire       regs_en1,
              output wire [2:0] regs_no1,
              input wire [7:0]  regs_out1,

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

   //
   // First part of the machinery is to decode the instruction
   // into opcode, dst-reg, src-regs and immediate values.
   //
   reg                          dest_valid;
   reg [2:0]                    dest_no;

   reg                          src0_valid;
   reg [2:0]                    src0_no;

   reg                          src1_valid;
   reg [2:0]                    src1_no;

   reg [3:0]                    opc_reg;
   reg [3:0]                    alu_reg;
   reg                          alu_valid;

   reg                          imm_valid;
   reg [7:0]                    imm_reg;

   always @(*)
     begin
        dest_valid <= 1'b0;
        dest_no <= 0;

        src0_valid <= 1'b0;
        src0_no <= 0;

        src1_valid <= 1'b0;
        src1_no <= 0;

        imm_valid <= 1'b0;
        imm_reg <= 0;

        opc_reg <= `C_HALT;
        alu_reg <= `OP_SHL;
        alu_valid <= 1'b0;

        if (enable)
          if (ir_reg[15:14] == 2'b01) begin
             dest_valid <= 1'b1;
             dest_no <= ir_reg[10:8];
             src0_valid <= 1'b1;
             src0_no <= ir_reg[10:8];

             opc_reg <= `C_ALU;
             alu_valid <= 1'b1;

             if (ir_reg[13:11] == 3'b000)
               alu_reg <= `OP_ADD;
             else if (ir_reg[13:11] == 3'b001)
               alu_reg <= `OP_SUB;
             else if (ir_reg[13:11] == 3'b011) begin
                alu_reg <= `OP_ADD;
                src0_valid <= 1'b0;
             end

             imm_valid <= 1'b1;
             imm_reg <= ir_reg[7:0];
          end else if (ir_reg[15:13] == 5'b100) begin
             // RR4 class
             opc_reg <= `C_ALU;
             alu_valid <= 1'b1;

             if (ir_reg[12:10] == 3'b000)
               alu_reg <= `OP_SHL;
             else if (ir_reg[12:10] == 3'b001)
               alu_reg <= `OP_LSR;
             else if (ir_reg[12:10] == 3'b010)
               alu_reg <= `OP_ASR;
             else if (ir_reg[12:10] == 3'b011)
               alu_reg <= `OP_ROR;
             else if (ir_reg[12:10] == 3'b100)
               alu_reg <= `OP_ADD;
             else if (ir_reg[12:10] == 3'b110)
               alu_reg <= `OP_SUB;
             else begin
                opc_reg <= `C_HALT;
                alu_valid <= 1'b0;
             end

             dest_valid <= 1'b1;
             dest_no <= ir_reg[9:7];
             src0_valid <= 1'b1;
             src0_no <= ir_reg[6:4];
             imm_valid <= 1'b1;
             imm_reg <= {4'b0000, ir_reg[3:0]};
          end else if (ir_reg[15:13] == 3'b110) begin
             // CALC (RRR) class
             alu_valid <= 1'b1;
             alu_reg <= ir_reg[12:9];
             dest_valid <= 1'b1;
             src0_valid <= 1'b1;
             src1_valid <= 1'b1;
             dest_no <= ir_reg[8:6];
             src0_no <= ir_reg[5:3];
             src1_no <= ir_reg[2:0];
          end else if (ir_reg[15:13] == 3'b111) begin
             if (ir_reg[11:8] == 4'b1110) begin
                opc_reg <= `C_OUT;
                src0_valid <= 1'b1;
                src0_no <= ir_reg[2:0];
             end else if (ir_reg[11:8] == 4'b0000) begin
                opc_reg <= `C_NOP;
             end
          end
     end // always @ *

   //
   // Now that we have the instruction decoded, we prepare for loading
   // the internal registers for the next stage.
   //

   assign regs_en0 = src0_valid;
   assign regs_no0 = src0_no;

   assign regs_en1 = src1_valid;
   assign regs_no1 = src1_no;

   always @(posedge clk)
     if (enable) begin
        o_dest_valid <= dest_valid;
        o_dest_no <= dest_no;

        o_src0_valid <= src0_valid;
        o_src0_no <= src0_no;

        o_src1_valid <= src1_valid;
        o_src1_no <= src1_no;

        if (src0_valid) begin
           o_src0_reg <= regs_out0;
        end
        if (src1_valid) begin
           o_src1_reg <= regs_out1;
        end
     end

   always @(posedge clk)
     if (enable)
       o_imm_valid <= imm_valid;

   always @(posedge clk)
     if (enable)
       o_imm_reg <= imm_reg;

   always @(posedge clk)
     if (enable)
       o_opc_reg <= opc_reg;

   always @(posedge clk)
     if (enable)
       o_alu_reg <= alu_reg;

   always @(posedge clk)
     if (enable)
       o_alu_valid <= alu_valid;

   // Jumps
   assign pc_i = 1'b0;
   assign pc_in = pc_i ? imm_reg : 0;
   assign pc_e = !pc_i && (opc_reg != `C_HALT);

endmodule // stage1


// ========= STAGE 2 ========
//
// - If ALU, then do the ALU and store to register file.
// - If store operation, then reg => mem.
// - If load operation, then mem => reg.
module stage2(clk, reset, enable,
              dest_valid, dest_no,
              src0_valid, src0_no, src0_reg,
              src1_valid, src1_no, src1_reg,
              imm_valid, imm_reg, opc_reg,
              alu_op, alu_valid,
              regs_wr, regs_no, regs_in,
              out_reg);

   input wire        clk;
   input wire        reset;
   input wire        enable;

   input wire        dest_valid;
   input wire [2:0]  dest_no;
   input wire        src0_valid;
   input wire [2:0]  src0_no;
   input wire [7:0]  src0_reg;
   input wire        src1_valid;
   input wire [2:0]  src1_no;
   input wire [7:0]  src1_reg;
   input wire        imm_valid;
   input wire [7:0]  imm_reg;
   input wire [3:0]  opc_reg;
   input wire [3:0]  alu_op;
   input wire        alu_valid;

   output wire       regs_wr; // Write enable
   output wire [2:0] regs_no; // Register number
   output wire [7:0] regs_in; // If regs_wr, then regs[regs_no] <= regs_in

   output reg [7:0]  out_reg;

   reg [7:0]         cache_reg;
   reg [2:0]         cache_no;
   reg               cache_valid;

   initial begin
      out_reg <= 8'h00;
      cache_valid <= 1'b0;
   end

   // The ALU unit
   wire [7:0]        alu_y;
   wire [7:0]        alu_s;
   wire [7:0]        alu_a;
   wire [7:0]        alu_b;

   wire [7:0]        src0_reg_in;
   wire [7:0]        src1_reg_in;

   alu alu(.a(alu_a), .b(alu_b), .y(alu_y),
           .op(alu_op), .z(), .ci(1'b0), .co());

   assign src0_reg_in = (cache_valid && src0_no == cache_no) ? cache_reg : src0_reg;
   assign src1_reg_in = (cache_valid && src1_no == cache_no) ? cache_reg : src1_reg;

   assign alu_a = alu_valid && src0_valid ? src0_reg_in : 8'h00;
   assign alu_b = alu_valid ? (src1_valid ? src1_reg_in :
                               imm_valid ? imm_reg : 8'h00) : 8'h00;

   assign regs_wr = alu_valid;
   assign regs_no = dest_valid ? dest_no : src0_no;
   assign regs_in = alu_valid ? alu_y : imm_valid ? imm_reg : 8'h00;

   always @(posedge clk)
     if (enable)
       if (regs_wr) begin
          cache_valid <= 1'b1;
          cache_reg <= regs_in;
          cache_no <= regs_no;
       end

   always @(posedge clk)
     if (enable && opc_reg == `C_OUT)
       out_reg <= src0_reg_in;

endmodule // stage2


module sap(input wire        clk,
           input wire        reset,
           input wire        enable,
           output wire [7:0] out_reg
           );
   // Startup control
   reg         enable_0;
   reg         enable_1;
   reg         enable_2;

   initial begin
      enable_0 = 1'b0;
      enable_1 = 1'b0;
      enable_2 = 1'b0;
   end

   always @(posedge clk)
     if (enable)
       begin
          enable_0 <= enable;
          enable_1 <= enable_0;
          enable_2 <= enable_1;
       end

   // Program Counter implemented as a 4-bit counter
   wire pc_e;
   wire pc_i;
   wire [7:0] pc_in; // If pc_i is true, pc_in is loaded as the new value.
   wire [7:0] pc_reg;
   counter8 pc(.clk(clk), .reset(reset),
               .enable(pc_e && enable_0), .load(pc_i && enable_0),
               .in(pc_in), .out(pc_reg));

   // Instruction register
   wire [15:0] ir_reg;

   // Address register
   wire        addr_i;
   wire [7:0]  addr_in; // If addr_i is true, addr_in is the new value next clock.
   wire [7:0]  addr_reg;

   reg8 ar(.load(addr_i && enable_0), .clk(clk), .reset(reset), .in(addr_in), .out(addr_reg));

   // Memory register
   reg         mem_i;    // If enabled: mem[AR] <= mem_in
   reg [7:0]   mem_in;
   wire [7:0]  mem_reg;  // Always:     mem_reg <= mem[AR]
   ram ram(.clk(clk),
           .enable(enable_0),
           .pc(pc_reg),
           .ir(ir_reg),
           .addr(addr_reg),
           .we(mem_i),
           .in(mem_in),
           .out(mem_reg)
           );

   // Register File
   wire        regs_en0;
   wire [2:0]  regs_no0;
   wire [7:0]  regs_out0;

   wire        regs_en1;
   wire [2:0]  regs_no1;
   wire [7:0]  regs_out1;

   wire        regs_wr;
   wire [2:0]  regs_no;
   wire [7:0]  regs_in;

   regs registers(.clk(clk),
                  .en0(regs_en0), .num0(regs_no0), .out0(regs_out0),
                  .en1(regs_en1), .num1(regs_no1), .out1(regs_out1),
                  .wr(regs_wr), .num(regs_no), .in(regs_in));

   wire        dest_valid;
   wire [2:0]  dest_no;

   wire        src0_valid;
   wire [2:0]  src0_no;
   wire [7:0]  src0_reg;

   wire        src1_valid;
   wire [2:0]  src1_no;
   wire [7:0]  src1_reg;

   wire [3:0]  opcode;
   wire [3:0]  alu_op;
   wire        alu_valid;

   wire        imm_valid;
   wire [7:0]  imm_reg;
   wire        zero_i;
   wire        zero_in;
   wire        zero_out;

   stage1 st1(.clk(clk),
              .reset(reset),
              .enable(enable_1),
              .ir_reg(ir_reg),
              // Pipes out
              .o_dest_valid(dest_valid),              .o_dest_no(dest_no),
              .o_src0_valid(src0_valid),              .o_src0_no(src0_no),              .o_src0_reg(src0_reg),
              .o_src1_valid(src1_valid),              .o_src1_no(src1_no),              .o_src1_reg(src1_reg),
              .o_opc_reg(opcode),                     .o_alu_valid(alu_valid),          .o_alu_reg(alu_op),
              .o_imm_valid(imm_valid),                .o_imm_reg(imm_reg),
              // Register bank
              .regs_en0(regs_en0),                    .regs_no0(regs_no0),              .regs_out0(regs_out0),
              .regs_en1(regs_en1),                    .regs_no1(regs_no1),              .regs_out1(regs_out1),
              // not here
              .zero_i(zero_i),
              .zero_in(zero_in),
              .zero_out(zero_out),
              .pc_i(pc_i),
              .pc_in(pc_in),
              .pc_e(pc_e)
              );

   stage2 st2(.clk(clk), .reset(reset), .enable(enable_2),
              .dest_valid(dest_valid), .dest_no(dest_no),
              .src0_valid(src0_valid), .src0_no(src0_no), .src0_reg(src0_reg),
              .src1_valid(src1_valid), .src1_no(src1_no), .src1_reg(src1_reg),
              .imm_valid(imm_valid), .imm_reg(imm_reg),
              .opc_reg(opcode), .alu_op(alu_op), .alu_valid(alu_valid),
              .regs_wr(regs_wr), .regs_no(regs_no), .regs_in(regs_in),
              .out_reg(out_reg));



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

   reg [23:0]                low_speed_ctr = 0;
   always @(posedge clk)
     low_speed_ctr <= low_speed_ctr + 1;

   wire                      slow_clk;
   wire [7:0]                out;

   assign slow_clk = low_speed_ctr[20];

   sap sap(.clk(slow_clk), .reset(1'b0), .enable(1'b1), .out_reg(out));

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

   sap DUT (.clk(clk), .reset(reset), .out_reg(out), .enable(!reset));

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

 `ifdef DETAILED
   initial begin
      $display("     PC   IR | IRp  OP D    S    Im   | OP  # D  # S     Im    W # WR ");
      #100 $finish;
   end

   always @(posedge clk) begin
      $strobe("%3d) %2h %4h | %4h  %h %1h %b  %1h %b  %2h %b |  %h  %h %b  %h %b-%02h  %b-%02h  %b %h %02h %08b | %02h %02h %02h | %02h",
              $time, DUT.pc_reg, DUT.ir_reg,

              DUT.st1.ir_reg, DUT.st1.opc_reg,
              DUT.st1.dest_no, DUT.st1.dest_valid,
              DUT.st1.src0_no, DUT.st1.src0_valid,
              DUT.st1.imm_reg, DUT.st1.imm_valid,

              DUT.st2.opc_reg,
              DUT.st2.dest_no, DUT.st2.dest_valid,
              DUT.st2.src0_no, DUT.st2.src0_valid, DUT.st2.src0_reg_in,
              DUT.st2.imm_valid, DUT.st2.imm_reg,
              DUT.st2.regs_wr, DUT.st2.regs_no, DUT.st2.regs_in, DUT.st2.regs_in,

              DUT.registers.regs[0],
              DUT.registers.regs[1],
              DUT.registers.regs[2],
              out);
   end


 `else // !`ifdef DETAILED
   initial begin
      $display("       A   B   O");
      $monitor("%04d) %h  %h  %h ", $time, DUT.o_dest_reg, DUT.o_src0_reg, out);
      //$monitor("%h", out);
      #1500 $finish;
   end // initial begin
 `endif

endmodule // tb

`endif
