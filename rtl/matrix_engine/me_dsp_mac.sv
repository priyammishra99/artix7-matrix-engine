// ============================================================================
// File: me_dsp_mac.sv
// Block: Matrix engine - one DSP48E1 multiply-accumulate-cascade stage
//
// The DSP48E1 primitive is instantiated directly so that the PCOUT -> PCIN
// cascade is guaranteed (inference left the chain adders in fabric).
//
//   P = ((A + D) * B) [+ PCIN]                                             
//
//   pins --> A1/B1 --> A2/B2, D --> pre-adder --> multiplier --> M --> ALU --> P
//           (stage 1)  (stage 2)                                (3)            (4)
//
// - Stationary operand (weight, K15 / V15 parts, scan code) enters the A side
//   (13 bits used of 25). D is the pre-adder input: 0 in GEMM mode and the
//   page scan, 15*mn in attention, so that A + D = K15 exactly.
// - Moving operand (activation, q16, p half, folded query) enters B (16 of 18).
// - A1/B1 (AREG = BREG = 2) are the last hop of the array's operand
//   distribution: each DSP has its own copy, so no fabric register fans out to
//   many DSPs. D has one register (DREG = 1); the array feeds it one cycle
//   later than A so that A2 and D meet in the pre-adder in the same cycle.
// - FIRST = 1 : OPMODE = 7'b000_0101  (X = Y = M, Z = 0)      chain start
//   FIRST = 0 : OPMODE = 7'b001_0101  (X = Y = M, Z = PCIN)   cascade
// - INMODE = 5'b00100: A2 and B2 paths, D port into the pre-adder, add.
//
// No resets or clock enables are used (all tied off), so the datapath is
// reset-free. Latency, pins A/B to P: 4 cycles (D to P: 3).
//
// Simulation needs the unisim library: xelab ... -L unisims_ver work.glbl
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_dsp_mac #(
  parameter bit FIRST = 1'b0
) (
  input  wire logic                clk,
  input  wire logic signed [24:0]  a,       // stationary operand
  input  wire logic signed [24:0]  d,       // pre-adder input (one cycle after a)
  input  wire logic signed [17:0]  b,       // moving operand
  input  wire logic [47:0]         pcin,    // from the previous DSP's PCOUT
  output wire logic signed [47:0]  p,       // P (used at the chain end)
  output wire logic [47:0]         pcout    // to the next DSP's PCIN
);

  localparam logic [6:0] OPM = FIRST ? 7'b000_0101 : 7'b001_0101;

  DSP48E1 #(
    .A_INPUT("DIRECT"), .B_INPUT("DIRECT"), .USE_DPORT("TRUE"),
    .USE_MULT("MULTIPLY"), .USE_SIMD("ONE48"),
    .AREG(2), .BREG(2), .DREG(1), .ADREG(0), .MREG(1), .PREG(1),
    .ACASCREG(2), .BCASCREG(2), .CREG(0), .CARRYINREG(0), .CARRYINSELREG(0),
    .ALUMODEREG(0), .INMODEREG(0), .OPMODEREG(0),
    .AUTORESET_PATDET("NO_RESET"), .MASK(48'h3fffffffffff), .PATTERN(48'h0),
    .SEL_MASK("MASK"), .SEL_PATTERN("PATTERN"), .USE_PATTERN_DETECT("NO_PATDET")
  ) u_dsp (
    .CLK(clk),
    .A({{5{a[24]}}, a}), .B(b), .C(48'd0), .D(d),
    .PCIN(pcin), .ACIN(30'd0), .BCIN(18'd0), .CARRYIN(1'b0), .CARRYCASCIN(1'b0),
    .MULTSIGNIN(1'b0),
    .OPMODE(OPM), .ALUMODE(4'b0000), .INMODE(5'b00100), .CARRYINSEL(3'b000),
    .CEA1(1'b1), .CEA2(1'b1), .CEB1(1'b1), .CEB2(1'b1), .CEC(1'b0), .CED(1'b1),
    .CEAD(1'b0), .CEM(1'b1), .CEP(1'b1), .CEALUMODE(1'b0), .CECTRL(1'b0),
    .CEINMODE(1'b0), .CECARRYIN(1'b0),
    .RSTA(1'b0), .RSTB(1'b0), .RSTC(1'b0), .RSTD(1'b0), .RSTALLCARRYIN(1'b0),
    .RSTALUMODE(1'b0), .RSTCTRL(1'b0), .RSTINMODE(1'b0), .RSTM(1'b0), .RSTP(1'b0),
    .P(p), .PCOUT(pcout),
    .ACOUT(), .BCOUT(), .CARRYOUT(), .CARRYCASCOUT(), .MULTSIGNOUT(),
    .OVERFLOW(), .UNDERFLOW(), .PATTERNDETECT(), .PATTERNBDETECT()
  );

endmodule

`default_nettype wire
