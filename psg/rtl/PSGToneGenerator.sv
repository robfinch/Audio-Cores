`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2007-2025  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
// PSGToneGenerator.v
//
// BSD 3-Clause License
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//                                                                          
//=============================================================================

module PSGToneGenerator(rst, clk, ack, test, vt, freq, pw, acc, pch_i, prev_acc, wave, sync, ringmod, fm_i, o);
input rst;
input clk;
input ack;              // wave table input acknowledge
input [15:0] wave;      // wave table data input
input test;
input [5:0] vt;         // voice type
input [21:0] freq;
input [21:0] pw;        // pulse width
input sync;
input ringmod;
input fm_i;
input [15:0] pch_i;
input  [31:0] prev_acc;  // from previous voice
output [31:0] acc;	     // 100MHz / 2^ 32 = 0.02328Hz resolution
output [15:0] o;

reg [31:0] accd;    // cycle delayed accumulator

integer n;

reg [15:0] outputT;
reg [15:0] outputW;
reg [22:0] lfsr;

wire synca = ~prev_acc[31]&acc[31]&sync;


PSGHarmonicSynthesizer u2
(
  .rst(rst),
  .clk(clk),
  .test(test),
  .sync(synca),
  .freq({10'h00,fm_i ? freq+pch_i : freq}),
  .o(acc)
);

// capture wave input
always_ff @(posedge clk)
if (rst)
  outputW <= 0;
else if (ack)
  outputW <= wave;

// Noise generator
always_ff @(posedge clk)
	if (acc[18] != acc[22])
		lfsr <= {lfsr[21:0],~(lfsr[22]^lfsr[17])};

// Triangle wave, ring modulation
wire msb = ringmod ? acc[31]^prev_acc[31] : acc[31];
always_ff @(posedge clk)
	outputT <= msb ? ~acc[30:19] : acc[30:19];

// Other waveforms, ho-hum
reg [15:0] outputP, outputS, outputN;
always_ff @(posedge clk) outputP = {16{acc[31:10] < pw}};
always_ff @(posedge clk) outputS = vt[5] ? ~acc[31:16] : acc[31:16];
always_ff @(posedge clk) outputN = lfsr[11:0];

wire [15:0] out;
PSGNoteOutMux #(16) u4 (.clk(clk), .s(vt[4:0]), .a(outputT), .b(outputS), .c(outputP), .d(outputN), .e(outputW), .o(out) );
assign o = out;

endmodule
