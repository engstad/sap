`timescale 1ms / 1us

module counter4(input enable,
                input load,
                input clk,
                input reset,
                input[3:0] in,
                output reg[3:0] out);
  always @(posedge clk)
    if (reset) begin
      out <= 4'b0;
    end else if (load) begin
      out <= in;
    end else if (enable) begin
      out <= out + 1;
    end
endmodule

module counter2(input clk, input reset, output reg[1:0] out);
  always @(posedge clk)
    if (reset)
      out <= 2'b0;
    else
      out <= out + 1;
endmodule

// A 4-bit register
module reg4(input load, input clk, input reset, input[3:0] in,
            output reg[3:0] out);
  always @(posedge clk)
    if (reset) begin
      out <= 4'b0;
    end else if (load) begin
      out <= in;
    end
endmodule

// An 8-bit register (implemented w/2 4-bit registers)
module reg8(input load, input load_hi, input clk, input reset, input[7:0] in,
            output wire[7:0] out);
  wire[3:0] lo_out, hi_out;
  wire[3:0] lo_in = in[3:0];
  wire[3:0] hi_in = load_hi ? in[3:0] : in[7:4];
  reg4 lo(.load(load), .clk(clk), .reset(reset),
          .in(lo_in), .out(lo_out));
  reg4 hi(.load(load | load_hi), .clk(clk), .reset(reset),
          .in(hi_in), .out(hi_out));
  assign out = {hi_out, lo_out};
endmodule

// The program ROM
module rom(input enable,
           input[3:0] addr,
           output reg[7:0] out);

  always @* begin
    if (enable)
      case (addr)
        4'b0000: out = 8'h6E; // 0: LAL #$E
        4'b0001: out = 8'h71; // 1: LAH #$1
        4'b0010: out = 8'h4E; // 2: LDB $E
        //4'b0011: out = 8'h50; // 3: ADD
        4'b0100: out = 8'h14; // 4: J #$4
        4'b1110: out = 8'h42; // E: .data $42
        4'b1111: out = 8'h1E; // F: .data $1E
        default: out = 8'h00;
      endcase
    else
      out = 8'hZZ;
  end
endmodule

// The micro-op ROM
module uop(input enable,
           input[5:0] addr,
           output reg[11:0] out);

  always @* begin
    if (!enable)
      out = 12'hZZZ;
    else
      case (addr)
        //                   OSBAAIIRMCCC
        //                   IOIIHOIOIEOI
        // NOP (0)
        6'b000000: out = 12'b000000001010; // MI CO
        6'b000001: out = 12'b000000110000; // II RO
        6'b000010: out = 12'b000000000000; //
        6'b000011: out = 12'b000000000100; // CE
        // JUMP (1)
        6'b000100: out = 12'b000000001010; // MI CO
        6'b000101: out = 12'b000000110000; // II RO
        6'b000110: out = 12'b000001000001; // CI IO
        6'b000111: out = 12'b000000000000; //
        // JUMP (2)
        6'b001000: out = 12'b000000001010; // MI CO
        6'b001001: out = 12'b000000110000; // II RO
        6'b001010: out = 12'b000001000001; // CI IO
        6'b001011: out = 12'b000000000000; //
        // LDA (3)
        6'b001100: out = 12'b000000001010; // MI CO
        6'b001101: out = 12'b000000110000; // II RO
        6'b001110: out = 12'b000001001000; // MI IO
        6'b001111: out = 12'b000100010100; // AI RO CE
        // LDB (4)
        6'b010000: out = 12'b000000001010; // MI CO      MAR <- PC
        6'b010001: out = 12'b000000111000; // II RO MI   IREG, MAR <- MEM
        6'b010010: out = 12'b001000010000; // BI RO
        6'b010011: out = 12'b110000100100; // SO OI CE
        // ADD (5)
        6'b010100: out = 12'b000000001010; // MI CO
        6'b010101: out = 12'b000000110000; // II RO
        6'b010110: out = 12'b110000000000; // OI SO
        6'b010111: out = 12'b000000000100; // CE
        // LAL (6) [Load A-Reg lower nibble]
        6'b011000: out = 12'b000000001010; // MI CO
        6'b011001: out = 12'b000000110000; // II RO
        6'b011010: out = 12'b000101000000; // AI IO
        6'b011011: out = 12'b000000000100; // CE
        // LAH (6) [Load A-Reg high nibble]
        6'b011100: out = 12'b000000001010; // MI CO
        6'b011101: out = 12'b000000110000; // II RO
        6'b011110: out = 12'b000011000000; // AI IO
        6'b011111: out = 12'b000000000100; // CE

        default:   out = 12'h00;
      endcase
  end
endmodule

module alu(input enable, input clk, input reset, input[7:0] a, input[7:0] b,
             output reg[7:0] out);
    always @(posedge clk) begin
      if (reset)
        out <= 8'h00;
      else if (enable)
        out <= a + b;
    end
endmodule

module buf8(input wire[7:0] x, input wire enable, output wire[7:0] y);
  assign y = enable ? x : 8'hZZ;
endmodule

module sap(input wire clk, input wire reset, output wire[7:0] out);
  localparam HI = 1'b1;
  localparam LO = 1'b0;

  // Control signals
  wire oi, so, bi, ai, ah, io, ii, ro, mi, ce, co, ci;

  // The bus
  wire[7:0] bus;

  // Program Counter implemented as a 4-bit counter
  wire[3:0] pc_out;
  counter4 pc(.enable(ce), .load(ci), .clk(clk), .reset(reset),
              .in(bus[3:0]), .out(pc_out));
  buf8 po({4'b0000, pc_out}, co, bus);

  // Memory Address Register as a 4-bit register
  wire[3:0] mar_out;
  reg4 mar(.load(mi), .clk(clk), .reset(reset), .in(bus[3:0]), .out(mar_out));

  // The program ROM is 4-bit addressable 8-bit memory.
  wire[7:0] mem_out;
  rom rom(.enable(ro), .addr(mar_out), .out(mem_out));
  buf8 memo(mem_out, ro, bus);

  // The Instruction Register
  wire[7:0] ireg_out;
  reg8 ireg(.load(ii), .load_hi(LO), .clk(clk), .reset(reset), .in(bus),
            .out(ireg_out));
  buf8 irego({4'b0000, ireg_out[3:0]}, io, bus);

  // The A Register (ALU input)
  wire[7:0] areg_out;
  reg8 areg(.load(ai), .load_hi(ah), .clk(clk), .reset(reset), .in(bus),
            .out(areg_out));

  // The B register (ALU input)
  wire[7:0] breg_out;
  reg8 breg(.load(bi), .load_hi(LO), .clk(clk), .reset(reset), .in(bus),
            .out(breg_out));

  // The ALU unit
  wire[7:0] alu_out;
  alu alu(.enable(HI), .clk(clk), .reset(reset), .a(areg_out), .b(breg_out),
          .out(alu_out));
  buf8 aluo(alu_out, so, bus);

  // The O (output) register
  reg8 oreg(.load(oi), .load_hi(LO), .clk(clk), .reset(reset), .in(bus),
            .out(out));

  // The micro-instruction counter
  wire[1:0] ucounter_out;
  counter2 ucounter(.clk(clk), .reset(reset), .out(ucounter_out));

  // And finally the micro-op ROM
  uop uop(.enable(HI), .addr({ireg_out[7:4], ucounter_out}),
          .out({oi, so, bi, ai, ah, io, ii, ro, mi, ce, co, ci}));
endmodule

module tb();
  reg clk, reset;
  wire[7:0] out;

  sap DUT (.clk(clk), .reset(reset), .out(out));

  initial begin
    clk = 1'b0;
    reset = 1'b1;
    repeat(4) #10 clk = ~clk;
    reset = 1'b0;
    forever #10 clk = ~clk;
  end

  initial begin
    $dumpfile("sap.vcd");
    $dumpvars;
  end

  initial begin
    $monitor("%3d: %02x", $time, out);
    #500 $finish;
  end
endmodule
