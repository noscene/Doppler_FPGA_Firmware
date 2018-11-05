
// based on https://github.com/gundy/tiny-synth/blob/develop/hdl/filter_svf.vh

module filter_svf16 (
  input  clk,
  input  signed [15:0] in,
  output signed [15:0] out_highpass,
  output signed [15:0] out_lowpass,
  output signed [15:0] out_bandpass,
  // output signed [SAMPLE_BITS-1:0] out_notch,
  input  signed [15:0] F,  /* F1: frequency control; fixed point 1.15  ; F = 2sin(π*Fc/Fs).  At a sample rate of 250kHz, F ranges from 0.00050 (10Hz) -> ~0.55 (22kHz) */
  input  signed [15:0] Q1  /* Q1: Q control;         fixed point 2.14  ; Q1 = 1/Q        Q1 ranges from 2 (Q=0.5) to 0 (Q = infinity). */
);


  reg signed[15:0] highpass;
  reg signed[15:0] lowpass;
  reg signed[15:0] bandpass;
  // reg signed[SAMPLE_BITS-1:0] notch;
  reg signed[15:0] in_sign_extended;

  localparam signed [15:0] MAX = (2**(15))-1;
  localparam signed [15:0] MIN = -(2**(15));

  `define CLAMP(x) ((x>MAX)?MAX:((x<MIN)?MIN:x[15:0]))

  assign out_highpass   = `CLAMP(highpass);
  assign out_lowpass    = `CLAMP(lowpass);
  assign out_bandpass   = `CLAMP(bandpass);
  // assign out_notch      = `CLAMP(notch);

  // intermediate values from multipliers
  reg signed [31:0] Q1_scaled_delayed_bandpass;
  reg signed [31:0] F_scaled_delayed_bandpass;
  reg signed [31:0] F_scaled_highpass;

  initial begin
    highpass  = 0;
    lowpass   = 0;
    bandpass  = 0;
    // notch     = 0;
  end

  // it can make audioglitches +  adding state machine
  // so TODO: check port this into SB_MAC16
  always @(posedge clk) begin
    in_sign_extended            = in;  /* sign-extend the input value to a wider precision */
    Q1_scaled_delayed_bandpass  = (bandpass * Q1) >>> 14;
    F_scaled_delayed_bandpass   = (bandpass * F) >>> 15;
    lowpass                     = lowpass + F_scaled_delayed_bandpass[15:0];
    highpass                    = in_sign_extended - lowpass - Q1_scaled_delayed_bandpass[15:0];
    F_scaled_highpass           = (highpass * F) >>> 15;
    bandpass                    = F_scaled_highpass[15:0] + bandpass;
    // notch                       = highpass + lowpass;
  end

endmodule
