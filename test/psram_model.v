// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Behavioral APS6404L PSRAM model (test code only, Rev B3 Task 21): a
// single-bit SPI slave, mode 0 (SI sampled on SCK rise, SO launched on SCK
// fall). Commands 0x66 (reset enable), 0x99 (reset, only right after 0x66),
// 0x02 (write) and 0x03 (read, no wait cycles), 24-bit address, MSB first,
// bursts wrap nothing (the address just increments). 128 KB of storage: the
// byte address is masked to 17 bits. Storage starts as a pseudo-random
// pattern (16-bit Galois LFSR).
//
// so is undriven while ce_n is high: the 2-state build outputs the last
// data value plus a so_driven flag (Verilator rejects 1'bz; the consumers
// mux on the flag), which behaves identically to the tri-state for the
// controller. so_dly_ps delays every change of the data and the flag
// alike (transport delay) to exercise the controller's MISO sample-phase
// pick.
//
// err (sticky) flags a protocol error: CE# low longer than 8 us (tCEM), an
// unknown command, or 0x99 without a preceding 0x66. err_code keeps the
// first one (1 = tCEM, 2 = unknown command, 3 = 0x99 without 0x66).
`default_nettype none
`timescale 1ns / 1ps
module psram_model (
    input  wire        sck,
    input  wire        ce_n,
    input  wire        si,
    output wire        so,
    output wire        so_driven,
    input  wire [31:0] so_dly_ps,
    output reg         err,
    output reg  [1:0]  err_code
);
  localparam integer MEM_BYTES = 131072;
  localparam real    TCEM_NS   = 8000.0;

  reg [7:0] mem [0:MEM_BYTES-1];

  integer    nbit;        // SCK rises seen in this CE# low window
  reg [7:0]  cmd;
  reg [23:0] addr;
  reg [7:0]  wbyte;
  reg        rst_en;      // the previous command was 0x66
  integer    ncmd;        // commands seen (debug)
  real       t_fall;
  reg        so_raw;      // undelayed SO value (2-state: the last data bit)
  reg        so_drv_raw;  // whether SO is driven (the read's data window)
  reg        so_d;
  reg        so_drv_d;

  integer    i;
  reg [15:0] lfsr;
  initial begin
    lfsr = 16'hACE1;
    for (i = 0; i < MEM_BYTES; i = i + 1) begin
      mem[i] = lfsr[7:0];
      lfsr = lfsr[0] ? ((lfsr >> 1) ^ 16'hB400) : (lfsr >> 1);
    end
    err = 1'b0;
    err_code = 2'd0;
    nbit = 0;
    cmd = 8'h00;
    addr = 24'h0;
    wbyte = 8'h00;
    rst_en = 1'b0;
    ncmd = 0;
    t_fall = 0.0;
    so_raw = 1'b0;
    so_d = 1'b0;
    so_drv_raw = 1'b0;
    so_drv_d = 1'b0;
  end

  task flag(input [1:0] code);
    begin
      if (!err) err_code = code;
      err = 1'b1;
    end
  endtask

  always @(negedge ce_n) begin
    nbit   = 0;
    t_fall = $realtime;
  end

  always @(posedge ce_n) begin
    so_drv_raw = 1'b0;                    // the SO goes undriven (was 1'bz)
    if ($realtime - t_fall > TCEM_NS) flag(2'd1);
  end

  // SI sampled on the SCK rise
  always @(posedge sck) if (!ce_n) begin
    if ($realtime - t_fall > TCEM_NS) flag(2'd1);
    if (nbit < 8) begin
      cmd = {cmd[6:0], si};
      if (nbit == 7) begin
        ncmd = ncmd + 1;
        case (cmd)
          8'h66: rst_en = 1'b1;
          8'h99: begin if (!rst_en) flag(2'd3); rst_en = 1'b0; end
          8'h02, 8'h03: rst_en = 1'b0;
          default: begin flag(2'd2); rst_en = 1'b0; end
        endcase
      end
    end else if (nbit < 32) begin
      addr = {addr[22:0], si};
    end else if (cmd == 8'h02) begin
      wbyte = {wbyte[6:0], si};
      if (((nbit - 32) % 8) == 7) begin
        mem[addr[16:0]] = wbyte;
        addr = addr + 24'd1;
      end
    end
    nbit = nbit + 1;
  end

  // SO launched on the SCK fall: after the 32nd rise of a 0x03, data bit
  // k = nbit - 32 (MSB first, byte after byte)
  always @(negedge sck) if (!ce_n && cmd == 8'h03 && nbit >= 32) begin
    so_raw = mem[(addr + (nbit - 32) / 8) & 24'h01FFFF][7 - ((nbit - 32) % 8)];
    so_drv_raw = 1'b1;                    // driving from the first data bit on
  end

  always @(so_raw) so_d <= #(so_dly_ps / 1000.0) so_raw;
  always @(so_drv_raw) so_drv_d <= #(so_dly_ps / 1000.0) so_drv_raw;
  assign so = so_d;
  assign so_driven = so_drv_d;
endmodule
`default_nettype wire
