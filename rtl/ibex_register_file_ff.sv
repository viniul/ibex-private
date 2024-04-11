// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also CREDITS.md.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * RISC-V register file
 *
 * Register file with 31 or 15x 32 bit wide registers. Register 0 is fixed to 0.
 * This register file is based on flip flops. Use this register file when
 * targeting FPGA synthesis or Verilator simulation.
 */
module enc_top (
    input [31:0] IN,
    output reg [38:0] OUT,
    input clk
);

    always @(*) begin
       OUT[31:0] <= IN[31:0];
       OUT[32] <= IN[0] ^ IN[1] ^ IN[2] ^ IN[3] ^ IN[4] ^ IN[5] ^ IN[6] ^ IN[7] ^ IN[8] ^ IN[13] ^ IN[17] ^ IN[26] ^ IN[27] ^ IN[29];
       OUT[33] <= IN[0] ^ IN[1] ^ IN[2] ^ IN[3] ^ IN[4] ^ IN[12] ^ IN[16] ^ IN[18] ^ IN[21] ^ IN[22] ^ IN[23] ^ IN[24] ^ IN[25] ^ IN[28];
       OUT[34] <= IN[0] ^ IN[5] ^ IN[6] ^ IN[7] ^ IN[8] ^ IN[11] ^ IN[15] ^ IN[18] ^ IN[19] ^ IN[21] ^ IN[22] ^ IN[30] ^ IN[31];
       OUT[35] <= IN[1] ^ IN[5] ^ IN[10] ^ IN[14] ^ IN[18] ^ IN[19] ^ IN[20] ^ IN[23] ^ IN[24] ^ IN[26] ^ IN[27] ^ IN[28] ^ IN[29] ^ IN[30];
       OUT[36] <= IN[2] ^ IN[6] ^ IN[9] ^ IN[14] ^ IN[15] ^ IN[16] ^ IN[17] ^ IN[19] ^ IN[20] ^ IN[21] ^ IN[23] ^ IN[25] ^ IN[29] ^ IN[31];
       OUT[37] <= IN[3] ^ IN[7] ^ IN[9] ^ IN[10] ^ IN[11] ^ IN[12] ^ IN[13] ^ IN[20] ^ IN[22] ^ IN[24] ^ IN[25] ^ IN[27] ^ IN[31];
       OUT[38] <= IN[4] ^ IN[8] ^ IN[9] ^ IN[10] ^ IN[11] ^ IN[12] ^ IN[13] ^ IN[14] ^ IN[15] ^ IN[16] ^ IN[17] ^ IN[26] ^ IN[28] ^ IN[30];
    end
endmodule

module corrector (input [38:0] IN,
    input [6:0] SYN,
    output reg [38:0] OUT
);

reg [38:0] LOC;

    always @(*) begin
       case (SYN)
           7'b0000111: LOC <= 39'h00_0000_0001;
           7'b0001011: LOC <= 39'h00_0000_0002;
           7'b0010011: LOC <= 39'h00_0000_0004;
           7'b0100011: LOC <= 39'h00_0000_0008;
           7'b1000011: LOC <= 39'h00_0000_0010;
           7'b0001101: LOC <= 39'h00_0000_0020;
           7'b0010101: LOC <= 39'h00_0000_0040;
           7'b0100101: LOC <= 39'h00_0000_0080;
           7'b1000101: LOC <= 39'h00_0000_0100;
           7'b1110000: LOC <= 39'h00_0000_0200;
           7'b1101000: LOC <= 39'h00_0000_0400;
           7'b1100100: LOC <= 39'h00_0000_0800;
           7'b1100010: LOC <= 39'h00_0000_1000;
           7'b1100001: LOC <= 39'h00_0000_2000;
           7'b1011000: LOC <= 39'h00_0000_4000;
           7'b1010100: LOC <= 39'h00_0000_8000;
           7'b1010010: LOC <= 39'h00_0001_0000;
           7'b1010001: LOC <= 39'h00_0002_0000;
           7'b0001110: LOC <= 39'h00_0004_0000;
           7'b0011100: LOC <= 39'h00_0008_0000;
           7'b0111000: LOC <= 39'h00_0010_0000;
           7'b0010110: LOC <= 39'h00_0020_0000;
           7'b0100110: LOC <= 39'h00_0040_0000;
           7'b0011010: LOC <= 39'h00_0080_0000;
           7'b0101010: LOC <= 39'h00_0100_0000;
           7'b0110010: LOC <= 39'h00_0200_0000;
           7'b1001001: LOC <= 39'h00_0400_0000;
           7'b0101001: LOC <= 39'h00_0800_0000;
           7'b1001010: LOC <= 39'h00_1000_0000;
           7'b0011001: LOC <= 39'h00_2000_0000;
           7'b1001100: LOC <= 39'h00_4000_0000;
           7'b0110100: LOC <= 39'h00_8000_0000;
           7'b0000001: LOC <= 39'h01_0000_0000;
           7'b0000010: LOC <= 39'h02_0000_0000;
           7'b0000100: LOC <= 39'h04_0000_0000;
           7'b0001000: LOC <= 39'h08_0000_0000;
           7'b0010000: LOC <= 39'h10_0000_0000;
           7'b0100000: LOC <= 39'h20_0000_0000;
           7'b1000000: LOC <= 39'h40_0000_0000;
           default: LOC <= 0;
        endcase
       OUT <= LOC ^ IN;
    end
endmodule


module dec_top (input [38:0] IN,
    output wire [31:0] OUT,
    output reg DBL,
    input clk
);

    wire [6:0] CHK;
    reg ERR;
    reg [6:0] SYN;
    assign CHK = IN[38:32];

    always @(*) begin
       SYN[0] <= IN[0] ^ IN[1] ^ IN[2] ^ IN[3] ^ IN[4] ^ IN[5] ^ IN[6] ^ IN[7] ^ IN[8] ^ IN[13] ^ IN[17] ^ IN[26] ^ IN[27] ^ IN[29] ^ CHK[0];
       SYN[1] <= IN[0] ^ IN[1] ^ IN[2] ^ IN[3] ^ IN[4] ^ IN[12] ^ IN[16] ^ IN[18] ^ IN[21] ^ IN[22] ^ IN[23] ^ IN[24] ^ IN[25] ^ IN[28] ^ CHK[1];
       SYN[2] <= IN[0] ^ IN[5] ^ IN[6] ^ IN[7] ^ IN[8] ^ IN[11] ^ IN[15] ^ IN[18] ^ IN[19] ^ IN[21] ^ IN[22] ^ IN[30] ^ IN[31] ^ CHK[2];
       SYN[3] <= IN[1] ^ IN[5] ^ IN[10] ^ IN[14] ^ IN[18] ^ IN[19] ^ IN[20] ^ IN[23] ^ IN[24] ^ IN[26] ^ IN[27] ^ IN[28] ^ IN[29] ^ IN[30] ^ CHK[3];
       SYN[4] <= IN[2] ^ IN[6] ^ IN[9] ^ IN[14] ^ IN[15] ^ IN[16] ^ IN[17] ^ IN[19] ^ IN[20] ^ IN[21] ^ IN[23] ^ IN[25] ^ IN[29] ^ IN[31] ^ CHK[4];
       SYN[5] <= IN[3] ^ IN[7] ^ IN[9] ^ IN[10] ^ IN[11] ^ IN[12] ^ IN[13] ^ IN[20] ^ IN[22] ^ IN[24] ^ IN[25] ^ IN[27] ^ IN[31] ^ CHK[5];
       SYN[6] <= IN[4] ^ IN[8] ^ IN[9] ^ IN[10] ^ IN[11] ^ IN[12] ^ IN[13] ^ IN[14] ^ IN[15] ^ IN[16] ^ IN[17] ^ IN[26] ^ IN[28] ^ IN[30] ^ CHK[6];

       ERR <= |SYN;
       DBL <= ~^SYN & ERR;
    end

wire [38:0] corrector_out;
assign OUT = corrector_out[31:0];

corrector corr_mod (.IN(IN), .SYN(SYN), .OUT(corrector_out));

endmodule

(* keep_hierarchy *)
module ibex_register_file_ff #(
  parameter bit                   RV32E             = 0,
  parameter int unsigned          DataWidth         = 32,
  parameter int unsigned          ECCDataWidth         = 39,
  parameter bit                   DummyInstructions = 0,
  parameter bit                   WrenCheck         = 0,
  parameter bit                   RdataMuxCheck     = 0,
  parameter logic [DataWidth-1:0] WordZeroVal       = '0
) (
  // Clock and Reset
  input  logic                 clk_i,
  input  logic                 rst_ni,

  input  logic                 test_en_i,
  input  logic                 dummy_instr_id_i,
  input  logic                 dummy_instr_wb_i,

  //Read port R1
  input  logic [4:0]           raddr_a_i,
  output logic [DataWidth-1:0] rdata_a_o,

  //Read port R2
  input  logic [4:0]           raddr_b_i,
  output logic [DataWidth-1:0] rdata_b_o,


  // Write port W1
  input  logic [4:0]           waddr_a_i,
  input  logic [DataWidth-1:0] wdata_a_i,
  input  logic                 we_a_i,

  // This indicates whether spurious WE or non-one-hot encoded raddr are detected.
  output logic                 err_o
);

  localparam int unsigned ADDR_WIDTH = RV32E ? 4 : 5;
  localparam int unsigned NUM_WORDS  = 2**ADDR_WIDTH;

  logic [ECCDataWidth-1:0] rf_reg   [NUM_WORDS];
  logic [NUM_WORDS-1:0] we_a_dec;

  logic oh_raddr_a_err, oh_raddr_b_err, oh_we_err;

  always_comb begin : we_a_decoder
    for (int unsigned i = 0; i < NUM_WORDS; i++) begin
      we_a_dec[i] = (waddr_a_i == 5'(i)) ? we_a_i : 1'b0;
    end
  end

  // SEC_CM: DATA_REG_SW.GLITCH_DETECT
  // This checks for spurious WE strobes on the regfile.
  if (WrenCheck) begin : gen_wren_check
    // Buffer the decoded write enable bits so that the checker
    // is not optimized into the address decoding logic.
    logic [NUM_WORDS-1:0] we_a_dec_buf;
    prim_buf #(
      .Width(NUM_WORDS)
    ) u_prim_buf (
      .in_i(we_a_dec),
      .out_o(we_a_dec_buf)
    );

    prim_onehot_check #(
      .AddrWidth(ADDR_WIDTH),
      .AddrCheck(1),
      .EnableCheck(1)
    ) u_prim_onehot_check (
      .clk_i,
      .rst_ni,
      .oh_i(we_a_dec_buf),
      .addr_i(waddr_a_i),
      .en_i(we_a_i),
      .err_o(oh_we_err)
    );
  end else begin : gen_no_wren_check
    logic unused_strobe;
    assign unused_strobe = we_a_dec[0]; // this is never read from in this case
    assign oh_we_err = 1'b0;
  end

  wire [38:0] encoded_wdata_a_i;
  enc_top w_encoder(wdata_a_i, encoded_wdata_a_i, clk);

  always @(posedge clk)
    if (wen) regs[~waddr[4:0]] <= encoded_wdata;

  wire due_1;
  dec_top d1_decoder(rf_reg[raddr_a_i], rdata_a_o, due1, clk);

  wire due_2;
  dec_top d2_decoder(rf_reg[raddr_b_i], rdata_b_o, due2, clk);

  // No flops for R0 as it's hard-wired to 0
  for (genvar i = 1; i < NUM_WORDS; i++) begin : g_rf_flops
    logic [ECCDataWidth-1:0] rf_reg_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        rf_reg_q <= WordZeroVal;
      end else if (we_a_dec[i]) begin
        rf_reg_q <= encoded_wdata_a_i;
      end
    end

    assign rf_reg[i] = rf_reg_q;
  end

  logic unused_dummy_instr;
  assign unused_dummy_instr = dummy_instr_id_i ^ dummy_instr_wb_i;

  // R0 is nil
  assign rf_reg[0] = WordZeroVal;

  assign oh_raddr_a_err = 1'b0;
  assign oh_raddr_b_err = 1'b0;

  assign err_o = oh_raddr_a_err || oh_raddr_b_err || oh_we_err;

  // Signal not used in FF register file
  logic unused_test_en;
  assign unused_test_en = test_en_i;

endmodule
