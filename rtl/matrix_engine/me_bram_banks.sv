// ============================================================================
// File: me_bram_banks.sv
// Block: Matrix engine - a deep memory as instantiated block RAM banks
//
// One write port, one read port, read data 2 cycles after rd (the RAM's
// address register, then its output register), built from NB block RAMs side
// by side in depth, so that Vivado does not choose the aspect ratio itself
// (the BRAM recount: inferred, the row store's staging takes 64 BRAM36
// instead of 36):
//   MODE 0: RAMB36E1, true dual port 1K x 36, 32 bits used (4 byte enables);
//           word a in RAM a / 1,024
//   MODE 1: RAMB36E1, simple dual port 512 x 72, 64 bits used (8 byte
//           enables); entry a in RAM a / 512
// USE_PRIM = 0: the same memory, behavioural (me_sdp_ram), for fast runs.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_bram_banks #(
  parameter int MODE     = 0,                    // 0: 1K x 32 banks; 1: 512 x 64 banks
  parameter int NB       = 36,                   // banks
  parameter int AW       = 16,                   // address width
  parameter bit USE_PRIM = 1'b1,
  localparam int W   = (MODE == 0) ? 32 : 64,
  localparam int BAW = (MODE == 0) ? 10 : 9       // address bits inside a bank
) (
  input  wire logic             clk,
  input  wire logic             we,
  input  wire logic [W/8-1:0]   wbe,
  input  wire logic [AW-1:0]    waddr,
  input  wire logic [W-1:0]     wdata,
  input  wire logic             rd,
  input  wire logic [AW-1:0]    raddr,
  output logic      [W-1:0]     rdata
);
  generate
    if (!USE_PRIM) begin : g_model
      me_sdp_ram #(.W(W), .DEPTH(NB << BAW), .AW(AW)) u_ram (
        .clk, .we, .wbe, .waddr, .wdata, .rd, .raddr, .rdata);
    end else begin : g_prim
      localparam int SW = AW - BAW;               // bank select width
      logic [SW-1:0] rb1, rb2;                    // the read's bank, through the 2-cycle latency
      logic [W-1:0]  rdo [NB];
      always_ff @(posedge clk) begin
        if (rd) rb1 <= raddr[AW-1:BAW];
        rb2 <= rb1;
      end
      for (genvar i = 0; i < NB; i++) begin : g_b
        wire logic wsel = we && (waddr[AW-1:BAW] == SW'(i));
        wire logic rsel = rd && (raddr[AW-1:BAW] == SW'(i));
        if (MODE == 0) begin : g_tdp
          logic [31:0] dob;
          RAMB36E1 #(
            .RAM_MODE("TDP"), .READ_WIDTH_A(0), .WRITE_WIDTH_A(36), .READ_WIDTH_B(36), .WRITE_WIDTH_B(0),
            .DOA_REG(0), .DOB_REG(1), .WRITE_MODE_A("WRITE_FIRST"), .WRITE_MODE_B("WRITE_FIRST"),
            .RAM_EXTENSION_A("NONE"), .RAM_EXTENSION_B("NONE"), .EN_ECC_READ("FALSE"), .EN_ECC_WRITE("FALSE"),
            .SIM_COLLISION_CHECK("NONE")
          ) u_bram (
            .CLKARDCLK(clk), .ENARDEN(wsel), .REGCEAREGCE(1'b0), .RSTRAMARSTRAM(1'b0), .RSTREGARSTREG(1'b0),
            .WEA(wbe), .ADDRARDADDR({1'b1, waddr[BAW-1:0], 5'b11111}),
            .DIADI(wdata), .DIPADIP(4'b0), .DOADO(), .DOPADOP(),
            .CLKBWRCLK(clk), .ENBWREN(rsel), .REGCEB(1'b1), .RSTRAMB(1'b0), .RSTREGB(1'b0),
            .WEBWE(8'b0), .ADDRBWRADDR({1'b1, raddr[BAW-1:0], 5'b11111}),
            .DIBDI(32'b0), .DIPBDIP(4'b0), .DOBDO(dob), .DOPBDOP(),
            .CASCADEINA(1'b0), .CASCADEINB(1'b0), .CASCADEOUTA(), .CASCADEOUTB(),
            .INJECTDBITERR(1'b0), .INJECTSBITERR(1'b0), .DBITERR(), .SBITERR(), .ECCPARITY(), .RDADDRECC()
          );
          assign rdo[i] = dob;
        end else begin : g_sdp
          logic [31:0] doa, dob;
          RAMB36E1 #(
            .RAM_MODE("SDP"), .READ_WIDTH_A(72), .WRITE_WIDTH_B(72), .READ_WIDTH_B(0), .WRITE_WIDTH_A(0),
            .DOA_REG(1), .DOB_REG(1), .RAM_EXTENSION_A("NONE"), .RAM_EXTENSION_B("NONE"),
            .EN_ECC_READ("FALSE"), .EN_ECC_WRITE("FALSE"), .SIM_COLLISION_CHECK("NONE")
          ) u_bram (
            // read: port A's address; data on DOADO (low 32) and DOBDO (high 32)
            .CLKARDCLK(clk), .ENARDEN(rsel), .REGCEAREGCE(1'b1), .RSTRAMARSTRAM(1'b0), .RSTREGARSTREG(1'b0),
            .WEA(4'b0), .ADDRARDADDR({1'b1, raddr[BAW-1:0], 6'b111111}), .DIADI(wdata[31:0]), .DIPADIP(4'b0),
            .DOADO(doa), .DOPADOP(),
            // write: port B's address; data on DIADI (low 32) and DIBDI (high 32), byte enables WEBWE
            .CLKBWRCLK(clk), .ENBWREN(wsel), .REGCEB(1'b1), .RSTRAMB(1'b0), .RSTREGB(1'b0),
            .WEBWE(wbe), .ADDRBWRADDR({1'b1, waddr[BAW-1:0], 6'b111111}),
            .DIBDI(wdata[63:32]), .DIPBDIP(4'b0), .DOBDO(dob), .DOPBDOP(),
            .CASCADEINA(1'b0), .CASCADEINB(1'b0), .CASCADEOUTA(), .CASCADEOUTB(),
            .INJECTDBITERR(1'b0), .INJECTSBITERR(1'b0), .DBITERR(), .SBITERR(), .ECCPARITY(), .RDADDRECC()
          );
          assign rdo[i] = {dob, doa};
        end
      end
      assign rdata = rdo[rb2];
    end
  endgenerate
endmodule

`default_nettype wire
