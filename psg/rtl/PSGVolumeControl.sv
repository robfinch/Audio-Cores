`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2007-2025  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
//	PSGVolumeControl.v 
//		Controls the PSG's output volume.
// The volume control is made non-linear with a ROM lookup table based on
// an increment similar to Fibonnaci series.
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

module PSGVolumeControl(rst_i, clk_i, i, volume, o);
input rst_i;
input clk_i;
input [21:0] i;
input [3:0] volume;
output [29:0] o;
reg [29:0] o;

// This ROM lookup table to delinearize the volume control.
// Uses mangled Fibonnaci series.
reg [7:0] vol;
always_comb
case(volume)
4'd00:  vol = 8'd00;
4'd01:  vol = 8'd01;
4'd03:  vol = 8'd01;
4'd04:  vol = 8'd02;
4'd05:  vol = 8'd03;
4'd06:  vol = 8'd05;
4'd07:  vol = 8'd08;
4'd09:  vol = 8'd13;
4'd10:  vol = 8'd21;
4'd11:  vol = 8'd34;
4'd12:  vol = 8'd56;
4'd13:  vol = 8'd90;
4'd14:  vol = 8'd151;   
4'd15:  vol = 8'd255;
endcase
always_ff @(posedge clk_i)
if (rst_i)
  o <= 26'b0;		// Force the output volume to zero on reset
else
  o <= i * vol;

endmodule
