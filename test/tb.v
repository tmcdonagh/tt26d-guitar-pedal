`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb #(parameter FILL_FRAMES = 64) ();   // cocotb RTL runs use the short delay fill horizon (the brief); the GL netlist keeps 32768

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // PSRAM (Rev B3 Task 21): the behavioral APS6404L model on the SPI pins
  // (uo[0] SCK, uo[1] CE#, uo[2] MOSI). uio[7] is the dedicated MISO
  // (Rev B3 final, DIP-safe pin map): the design sees the model's SO
  // while the model drives it, else the tb's uio_in[7] level. The ADC
  // data line is uio[1], driven by the tb reg adc_din (so the tests'
  // whole-vector uio_in writes cannot clobber it).
  // cocotb controls: psram_en (model fitted; 0 = absent: the model sees
  // CE# high), psram_so_dly_ps (SO delay, ps). psram_err = protocol error.
  reg         psram_en;
  reg  [31:0] psram_so_dly_ps;
  initial begin
    psram_en = 1'b0;
    psram_so_dly_ps = 32'd0;
  end
  wire        psram_so;
  wire        psram_so_driven;
  wire        psram_err;
  wire [1:0]  psram_err_code;
  reg         adc_din;
  initial adc_din = 1'b0;
  wire        miso_pin = psram_so_driven ? psram_so : uio_in[7];
  wire [7:0]  uio_pin  = {miso_pin, uio_in[6:2], adc_din, uio_in[0]};
  psram_model psram (.sck(uo_out[0]), .ce_n(uo_out[1] | ~psram_en), .si(uo_out[2]),
                     .so(psram_so), .so_driven(psram_so_driven), .so_dly_ps(psram_so_dly_ps),
                     .err(psram_err), .err_code(psram_err_code));
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  // Top module (renamed for this project):
`ifdef GL_TEST
  tt_um_tmcdonagh_guitar_pedal user_project (   // the flat netlist has no parameters
`else
  tt_um_tmcdonagh_guitar_pedal #(.FILL_FRAMES(FILL_FRAMES)) user_project (
`endif

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),    // Dedicated inputs (the DIP switches only)
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_pin),  // IOs: Input path (uio[7] PSRAM SO, uio[1] ADC data)
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

`ifndef GL_TEST
  // Testbench-only standalone shared multiplier (Task 16, test_mult_corners):
  // drives src/mult.v in isolation against a * b. Not part of the design.
  reg               mt_start;
  reg  signed [16:0] mt_a;
  reg         [11:0] mt_b;
  wire signed [28:0] mt_p;
  wire               mt_done;
  mult mult_tb (.clk(clk), .rst_n(rst_n), .start(mt_start), .a(mt_a), .b(mt_b),
                .p(mt_p), .done(mt_done));
`endif

endmodule
