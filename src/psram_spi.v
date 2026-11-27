// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// APS6404L PSRAM SPI engine (Rev B3, Task 21): single-bit SPI, mode 0,
// SCK = clk/2, 24-bit byte address = {8'h00, word, 1'b0} for the 15-bit
// word addresses, 16-bit data, MSB first.
//
// Sequence. After reset: wait 16 frames (8192 clk, >= 150 us), then the
// probe: 0x66 (reset enable), 0x99 (reset), write 0xA5C3 @ 0x010000 and
// 0x5A3C @ 0x010002 (outside the 64 KB ring), read both back sampling MISO
// with phase 0; if either word mismatches, read both again with phase 1.
// ok = 1 with the phase that passed, else ok = 0 (no PSRAM: ui[7] is the
// MISO pull level, the reads return 0x0000 / 0xFFFF). A reprobe pulse
// (delay-enable rising edge) clears ok and reruns the whole sequence from
// 0x66 once the bus is idle (a module hot-fitted on the bench is picked up).
// go (with ok, bus idle): write wdata @ waddr_w, then read raddr_w -> rdata.
// A go while busy is dropped. The inputs wdata / waddr_w / raddr_w are read
// bit by bit while the bits go out, so they must hold from go until busy
// falls. rdata is the read shift register: valid while !busy after a read.
//
// Transaction timing (E0 = the clk edge CE# falls, cnt = k after edge Ek):
// SCK rises at odd k, falls at even k; MOSI bit b is set at E(2b) and the
// PSRAM samples it at the rise E(2b+1). A read's data bit j is launched by
// the PSRAM at the fall E(64+2j); phase 0 samples it at the next rise
// E(65+2j) (half an SCK after launch), phase 1 one clk later at the fall
// E(66+2j) (a full SCK). The last fall is E(2N) (N = 8 or 48 SCK), CE#
// rises at E(2N+1): CE# low 17 clk (command) / 97 clk (read, write) =
// 48.5 SCK <= 49 SCK + 2 clk, 3.95 us at 24.576 MHz. CE# then stays high
// >= 2 clk (gap) before the next transaction.
//
// Bus sharing: hold (= the 595 shifter busy) gates the START of every
// transaction; one in flight finishes. busy covers a pending or running
// sequence and, combinationally, an accepted go, so the 595 shifter
// (bus_free = ~busy) can never start in the same clk CE# falls.
`default_nettype none
module psram_spi (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        go,
    input  wire        hold,
    input  wire [15:0] wdata,
    input  wire [14:0] waddr_w,
    input  wire [14:0] raddr_w,
    input  wire        miso,
    input  wire        reprobe,
    output reg         sck,
    output reg         cs_n,
    output reg         mosi,
    output reg  [15:0] rdata,
    output reg         ok,
    output wire        busy
);
  // ops: 0 = 0x66, 1 = 0x99, 2/3 = probe write word 0/1, 4/5 = probe read
  // word 0/1, 6 = write wdata @ waddr_w, 7 = read raddr_w
  reg [2:0]  op;
  reg        pu;        // power-up wait running (cnt counts 8192 clk)
  reg        rp;        // probe requested (power-up, reprobe)
  reg        pend;      // the op in `op` is waiting to start
  reg        act;       // CE# low: op running
  reg        gap;       // CE# high for its second clk
  reg        phase;     // MISO sample phase (see above)
  reg        bad;       // probe word 0 read back wrong
  reg [12:0] cnt;

  wire is_cmd = (op[2:1] == 2'b00);
  wire probe  = (op[2:1] != 2'b11);
  wire is_rd  = (op == 3'd4) | (op == 3'd5) | (op == 3'd7);
  wire is_wr  = ~is_cmd & ~is_rd;

  // probe pattern (0x5A3C = ~0xA5C3), also the read-back reference
  wire [15:0] pdat = {16{op[0]}} ^ 16'hA5C3;

  // MOSI bit b of the transaction (b = 0..47, MSB first):
  //   0..7 command, 8..15 address [23:16] = {7'b0, probe}, 16..31 address
  //   [15:0], 32..47 write data (0 for a read)
  wire [7:0]  cbyte = (op == 3'd0) ? 8'h66 : (op == 3'd1) ? 8'h99
                    : {7'b0000001, is_rd};
  wire [15:0] alo   = probe ? {14'd0, op[0], 1'b0}
                            : {(is_rd ? raddr_w : waddr_w), 1'b0};
  wire [15:0] dw    = probe ? pdat : wdata;
  wire [12:0] cnt_nx = cnt + 13'd1;
  wire [15:0] fld   = cnt_nx[6] ? (dw & {16{is_wr}}) : alo;
  wire [5:0]  b     = act ? cnt_nx[6:1] : 6'd0;     // bit on the line after this edge
  wire        bit_b = (b[5:4] == 2'b00) ? (b[3] ? (probe & (b[2:0] == 3'd7))
                                                : cbyte[~b[2:0]])
                    : fld[~b[3:0]];

  wire last  = is_cmd ? (cnt[6:0] == 7'd16) : (cnt[6:0] == 7'd96);
  wire start = pend & ~act & ~gap & ~hold;
  wire go_ok = go & ok & ~pend & ~act & ~gap & ~rp;
  wire seq   = rp & ~pu & ~pend & ~act & ~gap;      // a probe sequence begins
  // read data capture: pre-edge cnt 64..95 with cnt[0] == phase
  wire cap   = act & is_rd & (cnt[6:5] == 2'b10) & (cnt[0] == phase);
  wire mism  = (rdata != pdat);

  assign busy = act | pend | go_ok;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      op <= 3'd0; pu <= 1'b1; rp <= 1'b1; pend <= 1'b0; act <= 1'b0;
      gap <= 1'b0; phase <= 1'b0; bad <= 1'b0; cnt <= 13'd0;
      sck <= 1'b0; cs_n <= 1'b1; mosi <= 1'b0; rdata <= 16'd0; ok <= 1'b0;
    end else begin
      gap <= 1'b0;
      if (reprobe) begin
        rp <= 1'b1;
        ok <= 1'b0;
      end
      if (pu) begin
        cnt <= cnt_nx;
        if (&cnt) pu <= 1'b0;                       // cnt wraps to 0
      end
      if (seq) begin
        rp <= reprobe;     // a reprobe in this very clk queues one more
        op <= 3'd0; pend <= 1'b1; phase <= 1'b0; ok <= 1'b0;
      end
      if (go_ok) begin
        op <= 3'd6; pend <= 1'b1;
      end
      if (cap) rdata <= {rdata[14:0], miso};
      if (start) begin
        act <= 1'b1; pend <= 1'b0; cs_n <= 1'b0; cnt <= 13'd0;
        mosi <= bit_b;                              // b = 0 (act low)
      end else if (act) begin
        if (last) begin
          act <= 1'b0; cs_n <= 1'b1; gap <= 1'b1; cnt <= 13'd0;
          sck <= 1'b0; mosi <= 1'b0;
          // next op: 0..3 -> +1; 4 -> 5 (bad = word 0 wrong); 5 -> pass:
          // ok, fail: phase 0 -> retry the reads with phase 1, else give
          // up; 6 -> 7; 7 -> idle
          if (op == 3'd4) bad <= mism;
          if (op == 3'd5) begin
            if (!(bad | mism)) ok <= ~rp;           // (a reprobe since: rerun)
            else if (!phase) begin
              phase <= 1'b1; op <= 3'd4; pend <= 1'b1;
            end
          end else if (op != 3'd7) begin
            op <= op + 3'd1; pend <= 1'b1;
          end
        end else begin
          cnt <= cnt_nx;
          sck <= cnt_nx[0];
          mosi <= bit_b;
        end
      end
    end
  end
endmodule
`default_nettype wire
