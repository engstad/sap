`timescale 1ms / 1us

module counter4(input enable,
                input load,
                input clk,
                input reset,
                input[3:0] in,
                output reg[3:0] out);
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

`define NOP 4'h0
`define JMP 4'h1
`define LDA 4'h2
`define LDB 4'h3
`define LAL 4'h4
`define LAH 4'h5
`define ADD 4'h6

// The program ROM
module rom(input enable,
           input[3:0] addr,
           output reg[7:0] out);

  always @* begin
    if (enable)
      case (addr)
        //4'b0000: out = {`LAH, 4'h1}; // 0: LAL #$E
        //4'b0001: out = {`LAL, 4'hE}; // 1: LAH #$1
        4'b0000: out = {`LDA, 4'hF}; // 1: LAH #$1
        4'b0001: out = {`LDB, 4'hE}; // 2: LDB $E
        4'b0010: out = {`ADD, 4'h0}; // 3: ADD
        4'b0011: out = {`JMP, 4'h3}; // 4: J #$4
        4'b1110: out = 8'h42; // E: .data $42
        4'b1111: out = 8'h1E; // F: .data $1E
        default: out = 8'h00;
      endcase
    else
      out = 8'hZZ;
  end
endmodule

// The micro-op ROM
module ctrl(input[3:0] opcode,
            input clk,
            input reset,
            output reg[11:0] out);

  localparam CI = 1 << 0;
  localparam CO = 1 << 1;
  localparam CE = 1 << 2;
  localparam MI = 1 << 3;
  localparam MO = 1 << 4;
  localparam II = 1 << 5;
  localparam IO = 1 << 6;
  localparam AH = 1 << 7;
  localparam AI = 1 << 8;
  localparam BI = 1 << 9;
  localparam SO = 1 << 10;
  localparam OI = 1 << 11;

  reg [3:0] state;
  reg [3:0] next_state;

  parameter S0 = 4'b0001,
            S1 = 4'b0010,
            S2 = 4'b0100,
            S3 = 4'b1000;

  always @(negedge clk or posedge reset) begin
    if (reset) begin
      state <= S0;
    end else begin
      state <= next_state;
    end
  end

  always @* begin
    case (state)
      S0: begin next_state = S1; out = MI | CO; end
      S1: begin next_state = S2; out = II | MO; end
      default:
        case (opcode)
          `NOP: begin next_state = S0; out = CE; end
          `JMP: begin next_state = S0; out = CI | IO; end
          `LDA: case (state)
              S2: begin next_state = S3; out = MI | IO; end
              S3: begin next_state = S0; out = AI | MO | CE; end
             endcase
          `LDB: case (state)
              S2: begin next_state = S3; out = MI | IO; end
              S3: begin next_state = S0; out = BI | MO | CE; end
             endcase
          `ADD: begin next_state = S0; out = OI | SO | CE; end
          `LAL: begin next_state = S0; out = AI | IO | CE; end
          `LAH: begin next_state = S0; out = AH | IO | CE; end
          default: begin next_state = S0; out = CE; end
        endcase
    endcase
  end

endmodule

module alu(input[7:0] a, input[7:0] b, output [7:0] out);
    assign out = a + b;
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
  alu alu(.a(areg_out), .b(breg_out),
          .out(alu_out));
  buf8 sreg(alu_out, so, bus);

  // The O (output) register
  reg8 oreg(.load(oi), .load_hi(LO), .clk(clk), .reset(reset), .in(bus),
            .out(out));

  // And finally the control unit
  ctrl ctrl(.clk(clk), .reset(reset), .opcode(ireg_out[7:4]),
            .out({oi, so, bi, ai, ah, io, ii, ro, mi, ce, co, ci}));
endmodule

`ifdef TEST
module tb();
  reg clk, reset;
  wire[7:0] out;

  sap DUT (.clk(clk), .reset(reset), .out(out));

  initial begin
    clk = 1'b1;
    reset = 1'b1;
    #10 reset = 1'b0;
  end

  initial begin
    forever #5 clk = ~clk;
  end

  initial begin
    $dumpfile("sap.vcd");
    $dumpvars;
  end

  initial begin
    $display("time A  B  O");
    $monitor("%3d: %h %h %h", $time, DUT.areg.out, DUT.breg.out, out);
    #500 $finish;
  end
endmodule
`endif
