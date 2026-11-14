// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Control plane for the silicon pedalboard: EC11 encoder quadrature
// decode with per-line bounce filtering, latching stomp bypass, preset
// counter + effect-enable ROM, and preset digit for the 7-seg display.
// Panel lines are sampled once per audio frame on tick.

`default_nettype none

module pedal_panel (
    input  wire       clk,        // 24.576 MHz
    input  wire       rst_n,
    input  wire       tick,       // 1-clk pulse once per audio frame (sample_valid)
    input  wire [2:0] panel_in,   // {stomp ui[3], enc_b ui[2], enc_a ui[1]}
    output wire [3:0] preset_en,  // ROM[preset]: en[0]=fuzz..en[3]=trem
    output wire       bypass,     // latching stomp toggle
    output wire [6:0] seg         // active-high a..g on seg[0..6]; 7'h00 = blank
);

  wire enc_a = panel_in[0];
  wire enc_b = panel_in[1];
  wire stomp = panel_in[2];

  // ------------------------------------------------------------------
  // Input sampling + bounce filter (per line): a level change is
  // accepted only after 48 consecutive equal frame samples (~1 ms at
  // the 48 kHz frame rate). *_s hold the last sampled level; *_cnt
  // count the equal-sample run (reset to 0 on change, saturating at
  // 47); the run's 48th sample latches the level into *_fil.
  // ------------------------------------------------------------------
  reg       a_s,   b_s,   st_s;
  reg [5:0] a_cnt, b_cnt, st_cnt;
  reg       a_fil, b_fil, st_fil;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_s <= 1'b0;   a_cnt <= 6'd0;   a_fil <= 1'b0;
      b_s <= 1'b0;   b_cnt <= 6'd0;   b_fil <= 1'b0;
      st_s <= 1'b0;  st_cnt <= 6'd0;  st_fil <= 1'b0;
    end else if (tick) begin
      // encoder A
      if (enc_a != a_s) begin
        a_s   <= enc_a;
        a_cnt <= 6'd0;
      end else if (a_cnt == 6'd46) begin
        a_cnt <= 6'd47;                       // run saturated: 48 samples
        a_fil <= a_s;                         // accept the level
      end else if (a_cnt != 6'd47) begin
        a_cnt <= a_cnt + 6'd1;
      end
      // encoder B
      if (enc_b != b_s) begin
        b_s   <= enc_b;
        b_cnt <= 6'd0;
      end else if (b_cnt == 6'd46) begin
        b_cnt <= 6'd47;
        b_fil <= b_s;
      end else if (b_cnt != 6'd47) begin
        b_cnt <= b_cnt + 6'd1;
      end
      // stomp
      if (stomp != st_s) begin
        st_s   <= stomp;
        st_cnt <= 6'd0;
      end else if (st_cnt == 6'd46) begin
        st_cnt <= 6'd47;
        st_fil <= st_s;
      end else if (st_cnt != 6'd47) begin
        st_cnt <= st_cnt + 6'd1;
      end
    end
  end

  // ------------------------------------------------------------------
  // Accepted-edge detect: *_prev delay *_fil by one clk, so a newly
  // accepted level change is a single-clk edge pulse.
  // ------------------------------------------------------------------
  reg a_prev, st_prev;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_prev  <= 1'b0;
      st_prev <= 1'b0;
    end else begin
      a_prev  <= a_fil;
      st_prev <= st_fil;
    end
  end

  wire a_rose  = a_fil & ~a_prev;
  wire a_fell  = ~a_fil & a_prev;
  wire st_rose = st_fil & ~st_prev;

  // ------------------------------------------------------------------
  // Preset counter (mod 8) + bypass latch. Quadrature is x2 on accepted
  // A edges only, direction from the accepted B level: rose -> B low +1
  // / B high -1; fell -> B high +1 / B low -1 (-1 = +7 mod 8). The
  // stomp latch toggles on each accepted press edge and holds through
  // release (latching, not momentary).
  // ------------------------------------------------------------------
  reg [2:0] preset_cnt;
  reg       bypass_q;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      preset_cnt <= 3'd0;
      bypass_q   <= 1'b0;
    end else begin
      if (a_rose)
        preset_cnt <= preset_cnt + (b_fil ? 3'd7 : 3'd1);
      else if (a_fell)
        preset_cnt <= preset_cnt + (b_fil ? 3'd1 : 3'd7);
      if (st_rose)
        bypass_q <= ~bypass_q;
    end
  end

  // ------------------------------------------------------------------
  // Preset ROM (provisional patterns): en[0]=fuzz, en[1]=dist,
  // en[2]=crush, en[3]=trem — chain order = bit order.
  // ------------------------------------------------------------------
  reg [3:0] rom_en;
  always @* begin
    case (preset_cnt)
      3'd0:    rom_en = 4'b0000;
      3'd1:    rom_en = 4'b0001;
      3'd2:    rom_en = 4'b0010;
      3'd3:    rom_en = 4'b0100;
      3'd4:    rom_en = 4'b1000;
      3'd5:    rom_en = 4'b1100;
      3'd6:    rom_en = 4'b1101;
      3'd7:    rom_en = 4'b1111;
      default: rom_en = 4'b0000;
    endcase
  end

  // ------------------------------------------------------------------
  // Preset digit, BCD -> 7-seg (active high, seg[0]=a .. seg[6]=g).
  // Blank while the bypass latch is engaged.
  // ------------------------------------------------------------------
  reg [6:0] seg_pat;
  always @* begin
    case (preset_cnt)
      3'd0:    seg_pat = 7'h3F;
      3'd1:    seg_pat = 7'h06;
      3'd2:    seg_pat = 7'h5B;
      3'd3:    seg_pat = 7'h4F;
      3'd4:    seg_pat = 7'h66;
      3'd5:    seg_pat = 7'h6D;
      3'd6:    seg_pat = 7'h7D;
      3'd7:    seg_pat = 7'h07;
      default: seg_pat = 7'h00;
    endcase
  end

  assign preset_en = rom_en;
  assign bypass    = bypass_q;
  assign seg       = bypass_q ? 7'h00 : seg_pat;

endmodule

`default_nettype wire
