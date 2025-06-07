`timescale 1ns / 1ps
// ============================================================================
//        __
//   \\__/ o\    (C) 2007-2025  Robert Finch, Waterloo
//    \  __ /    All rights reserved.
//     \/_//     robfinch<remove>@finitron.ca
//       ||
//
// PSG32.v
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
//
//	Registers
//  00      -------- --ffffff ffffffff ffffffff     freq [21:0]
//  04      -------- --pppppp pppppppp pppppppp     pulse width
//	08	    R------- ----oooo trsg-ef- vvvvvv-- 	test, ringmod, sync, gate, filter, output, voice type
//  0C      ---aaaaa aaaaaaaa aaaaaaaa aaaaaaaa     attack
//  10      --dddddd dddddddd dddddddd dddddddd     decay
//  14      -------- -------- -------- ssssssss     sustain / wave volume
//  18      aaaaaaaa rrrrrrrr rrrrrrrr rrrrrrrr     release / wave table buffer length
//  1C      aaaaaaaa aaaaaaaa aaaaaaaa aaa-----     wave table base address
//											vvvvv
//											wnpst
//  20-3C   Voice #2
//  40-5C   Voice #3
//  60-7C   Voice #4
//  80-9C   Voice #5
//  A0-BC   Voice #6
//  C0-DC   Voice #7
//  E0-FC   Voice #8
// 100-11C	Input
//
//	...
//	120     -------- -------- -------- ----vvvv   volume (0-15)
//	124     nnnnnnnn nnnnnnnn nnnnnnnn nnnnnnnn   osc3 oscillator 3
//	128     -------- -------- -------- nnnnnnnn   env[3] envelope 3
//  12C     -sss-sss -sss-sss -sss-sss -sss-sss   env state
//  130     ----oooo -------- RRRRRRRR RRRRRRRR   filter sample rate clock divider, output
//	134			-------- -------- -------i oooooooo		interrupt enable
//	138			-------- -------- -------i oooooooo		interrupt occurred
//	13C			-------- -------- -------i oooooooo		playback ended
//	140			-------- -------- -------i oooooooo		channel sync
//
//  180-1F8   -------- -------- s---kkkk kkkkkkkk   filter coefficients
//
//=============================================================================

//`define BUS_WID8    1'b1
`define BUS_WID     32

module PSG32(rst_i, clk_i, clk100_i, cs_i, 
	cyc_i, stb_i, ack_o, rdy_o, we_i, adr_i, dat_i, dat_o,
	m_cyc_o, m_stb_o, m_ack_i, m_we_o, m_sel_o, m_adr_o, m_dat_i, m_dat_o,
	aud_i, aud_o
);
parameter BUS_WID = 32;
parameter NVOICE = 8;
parameter MDW = 256;
// WISHBONE SYSCON
input rst_i;
input clk_i;			// system bus clock
input clk100_i;          // 100MHz reference clock
// NON-WISHBONE
input cs_i;             // circuit select
// WISHBONE SLAVE
input cyc_i;			// cycle valid
input stb_i;			// data strobe
output ack_o;
output rdy_o;
input we_i;				// write
input [8:0] adr_i;		// address input
input [BUS_WID-1:0] dat_i;		// data input
output reg [BUS_WID-1:0] dat_o;	// data output

// WISHBONE MASTER
output reg m_cyc_o;
output reg m_stb_o;
output reg m_we_o;
output reg [MDW/8-1:0] m_sel_o;
input m_ack_i;
output reg [31:0] m_adr_o;	// wave table address
input  [MDW-1:0] m_dat_i;	// wave table data input
output reg [MDW-1:0] m_dat_o;

input [15:0] aud_i;
output reg [15:0] aud_o [0:3];


genvar g;
integer n1,n2;

typedef enum logic [2:0]
{
	ST_WAIT = 3'd0,
	ST_FETCH,
	ST_ACK
} wave_state_e;

wave_state_e [8:0] wave_state;

reg [3:0] master;
reg [NVOICE:0] test;				// test (enable note generator)
reg [NVOICE:0] srst;        // soft reset
reg [5:0] vt [0:NVOICE];		// voice type
reg [21:0] freq [0:NVOICE];	// frequency control
reg [21:0] pw [0:NVOICE];		// pulse width control
reg [NVOICE:0] gate;
reg [28:0] attack [0:NVOICE];
reg [23:0] decay [0:NVOICE];
reg [7:0] sustain [0:NVOICE];
reg [31:0] relese [0:NVOICE];
reg [31:0] wtadr [0:NVOICE];
reg [31:0] wta_offs [0:NVOICE];
reg [NVOICE:0] autorep;
reg [NVOICE:0] sync;
reg [NVOICE:0] ringmod;
reg [NVOICE:0] fm;
reg [3:0] outctrl[0:NVOICE] ;
reg [15:0] crd;                 // clock rate divider for filter sampling
reg [NVOICE:0] filt;                 // 1 = output goes to filter
reg [NVOICE:0] eg;                   // 1 = output goes through envelope generator
wire [31:0] acc [0:NVOICE];
reg [3:0] volume;	// master volume
wire [15:0] tg_o [0:NVOICE];    	// tone generator output
wire [7:0] env [0:NVOICE];				// envelope generator output
wire [19:0] out [0:NVOICE];
wire [2:0] es[0:NVOICE];
wire [29:0] out4;
reg [21:0] sum [0:3];
reg [21:0] fsum;
reg [21:0] sum2 [0:3];
reg [3:0] filt_osel;
wire [21:0] filtin1;	// FIR filter input
wire [38:0] filt_o;		// FIR filter output
reg [NVOICE:0] playback_ended;

reg [15:0] wave_i [0:8];
reg [8:0] wave_request;
reg [8:0] wave_we;
reg [31:0] wave_addr [0:8];
reg [8:0] wave_ack;
wire [8:0] wave_sel;

and(cs, cyc_i, stb_i, cs_i);
reg ack1,ack2;
always_ff @(posedge clk_i)
	ack1 <= cs;
always_ff @(posedge clk_i)
    ack2 <= ack1 & cs;
assign ack_o = cs ? (we_i ? 1'b1 : ack2) : 1'b0;
assign rdy_o = cs ? (we_i ? 1'b1 : ack2) : 1'b1;

// Register shadow ram for register readback
`ifdef BUS_WID8
reg [7:0] reg_shadow [511:0];
reg [8:0] radr;
always_ff @(posedge clk_i)
  if (cs & we_i)  reg_shadow[adr_i[8:0]] <= dat_i;
always_ff @(posedge clk_i)
  radr <= adr_i[8:0];
wire [7:0] reg_shadow_o = reg_shadow[radr];
`else
reg [31:0] reg_shadow [127:0];
reg [8:0] radr;
always @(posedge clk_i)
  if (cs & we_i)  reg_shadow[adr_i[8:2]] <= dat_i;
always @(posedge clk_i)
  radr <= adr_i[8:2];
wire [31:0] reg_shadow_o = reg_shadow[radr];
`endif

// write to registers
always_ff @(posedge clk_i)
begin
	if (rst_i) begin
		for (n1 = 0; n1 < NVOICE; n1 = n1 + 1) begin
			freq[n1] <= 22'd0;
			pw[n1] <= 22'd0;
			test[n1] <= 1'b0;
			vt[n1] <= 6'd0;
			gate[n1] <= 1'b0;
			attack[n1] <= 29'd0;
			decay[n1] <= 24'd0;
			sustain[n1] <= 8'd0;
			relese[n1] <= 24'd0;
			filt[n1] <= 1'b0;
			sync[n1] <= 1'b0;
			ringmod[n1] <= 1'b0;
			fm[n1] <= 1'b0;
    		outctrl[n1] <= 0;
		end
		volume <= 0;
		crd <= 1000;
	end
	else begin
		if (cs & we_i) begin
	    if (BUS_WID==8) begin
        casez(adr_i[8:0])
        //---------------------------------------------------------
        9'b0???00000:	freq[adr_i[7:5]][7:0] <= dat_i;
        9'b0???00001: freq[adr_i[7:5]][15:8] <= dat_i;
        9'b0???00010: freq[adr_i[7:5]][19:16] <= dat_i[3:0];
        9'b0???00100: pw[adr_i[7:5]][7:0] <= dat_i;
        9'b0???00101: pw[adr_i[7:5]][15:8] <= dat_i;
        9'b0???01000:
        	begin
        		autorep[adr_i[7:5]] <= dat_i[0];
        		vt[adr_i[7:5]] <= dat_i[7:2];
        	end
        9'b0???01001:
        	begin
            filt[adr_i[7:5]] <= dat_i[1];
            eg[adr_i[7:5]] <= dat_i[2];
            fm[adr_i[7:5]] <= dat_i[3];
            gate[adr_i[7:5]] <= dat_i[4];
            sync[adr_i[7:5]] <= dat_i[5];
            ringmod[adr_i[7:5]] <= dat_i[6]; 
            test[adr_i[7:5]] <= dat_i[7];
          end
        9'b0???01010: outctrl[adr_i[7:5]] <= dat_i[3:0];
        9'b0???01011: srst[adr_i[7:5]] <= dat_i[7];
        9'b0???01100: attack[adr_i[7:5]][7:0] <= dat_i;
        9'b0???01101: attack[adr_i[7:5]][15:8] <= dat_i;
        9'b0???01110: attack[adr_i[7:5]][23:16] <= dat_i;
        9'b0???01111: attack[adr_i[7:5]][28:24] <= dat_i[4:0];
        
        9'b0???10000: decay[adr_i[7:5]][7:0] <= dat_i;
        9'b0???10001: decay[adr_i[7:5]][15:8] <= dat_i;
        9'b0???10010:	decay[adr_i[7:5]][23:16] <= dat_i;
        9'b0???10100:	sustain[adr_i[7:5]] <= dat_i;
        9'b0???11000:	relese[adr_i[7:5]][7:0] <= dat_i;
        9'b0???11001:	relese[adr_i[7:5]][15:8] <= dat_i;
        9'b0???11010:	relese[adr_i[7:5]][23:16] <= dat_i;
        9'b0???11100:  wtadr[adr_i[7:5]][7:0] <= {dat_i[7:1],1'b0};
        9'b0???11101:  wtadr[adr_i[7:5]][15:8] <= dat_i;
        9'b0???11110:  wtadr[adr_i[7:5]][23:16] <= dat_i;
        9'b0???11111:  wtadr[adr_i[7:5]][31:24] <= dat_i;
        //---------------------------------------------------------
				9'hB0:	volume <= dat_i[3:0];
        9'hC0:  crd[7:0] <= dat_i;
        9'hC1:  crd[15:8] <= dat_i;
        9'hC3:	filt_osel <= dat_i[3:0];
        default:	;
        endcase
	    end
	    else if (BUS_WID==32) begin
				casez(adr_i[8:2])
				//---------------------------------------------------------
				7'b0???000:	freq[adr_i[7:5]] <= dat_i[19:0];
				7'b0???001:	pw[adr_i[7:5]] <= dat_i[15:0];
				7'b0???010:
					begin
						autorep[adr_i[7:5]] <= dat_i[0];
						vt[adr_i[7:5]] <= dat_i[7:2];
						filt[adr_i[7:5]] <= dat_i[9];
						eg[adr_i[7:5]] <= dat_i[10];
						fm[adr_i[7:5]] <= dat_i[11];
						gate[adr_i[7:5]] <= dat_i[12];
						sync[adr_i[7:5]] <= dat_i[13];
						ringmod[adr_i[7:5]] <= dat_i[14]; 
						test[adr_i[7:5]] <= dat_i[15];
						outctrl[adr_i[7:5]] <= dat_i[19:16];
						srst[adr_i[7:5]] <= dat_i[31];
					end
				7'b0???011:	attack[adr_i[7:5]] <= dat_i[28:0];
				7'b0???100: decay[adr_i[7:5]] <= dat_i[23:0];
			  7'b0???101: sustain[adr_i[7:5]] <= dat_i[7:0];
				7'b0???110: relese[adr_i[7:5]] <= dat_i[31:0];
	      7'b0???111: wtadr[adr_i[7:5]] <= {dat_i[31:5],5'b0};
	               
				7'b1000000:	freq[adr_i[7:5]] <= dat_i[19:0];
				7'b1000001:	pw[adr_i[7:5]] <= dat_i[15:0];
				7'b1000010:
					begin
						autorep[8] <= dat_i[0];
						vt[8] <= dat_i[7:2];
						filt[8] <= dat_i[9];
						eg[8] <= dat_i[10];
						fm[8] <= dat_i[11];
						gate[8] <= dat_i[12];
						sync[8] <= dat_i[13];
						ringmod[8] <= dat_i[14]; 
						test[8] <= dat_i[15];
						outctrl[8] <= dat_i[19:16];
						srst[8] <= dat_i[31];
					end
				7'b1000011:	attack[8] <= dat_i[28:0];
				7'b1000100: decay[8] <= dat_i[23:0];
			  7'b1000101: sustain[8] <= dat_i[7:0];
				7'b1000110: relese[8] <= dat_i[31:0];
	      7'b1000111: wtadr[8] <= {dat_i[31:5],5'b0};
				//---------------------------------------------------------
				7'h48:	volume <= dat_i[3:0];
				7'h4C:  
					begin
						crd <= dat_i[15:0];
						filt_osel <= dat_i[27:24];
					end

				default:	;
				endcase
			end
		end
	end
end

always_ff @(posedge clk_i)
  if (BUS_WID==8) begin
  	if (cs)
	    case(adr_i[8:0])
	    9'h124:  dat_o <= acc[NVOICE-1][7:0];
	    9'h125:  dat_o <= acc[NVOICE-1][15:8];
	    9'h126:  dat_o <= acc[NVOICE-1][23:16];
	    9'h127:  dat_o <= acc[NVOICE-1][31:24];
	    9'h128:  dat_o <= env[NVOICE-1];
	    9'h12C:  dat_o <= {1'b0,es[1],1'b0,es[0]};
	    9'h12D:  dat_o <= {1'b0,es[3],1'b0,es[2]};
	    9'h12E:  dat_o <= {1'b0,es[5],1'b0,es[4]};
	    9'h12F:  dat_o <= {1'b0,es[7],1'b0,es[6]};
	    default:
	      dat_o <= reg_shadow_o;
	    endcase
	  else
	  	dat_o <= 8'h00;
  end
  else begin
  	if (cs)
	    case(adr_i[8:2])
	    7'h48:  dat_o <= acc[NVOICE-1];
	    7'h49:	dat_o <= {24'h0,env[NVOICE-1]};
	    7'h4A:  dat_o <= {1'b0,es[7],1'b0,es[6],1'b0,es[5],1'b0,es[4],1'b0,es[3],1'b0,es[2],1'b0,es[1],1'b0,es[0]};
	    default:
	      dat_o <= reg_shadow_o;
	    endcase
	 	else
	 		dat_o <= 32'h0;
  end

wire [3:0] wave_senc;

RoundRobinArbiter #(9)
urr1
(
  .rst(rst_i),
  .clk(clk_i),
  // If nothing is selected yet, or if there is an ack back for the selected
  .ce(wave_sel==9'd0 || m_ack_i),
  .hold(1'b1),
	.req(wave_request),
  .grant(wave_sel),
  .grant_enc(wave_senc)
);

reg [MDW-1:0] sampl1 [0:8];
reg [MDW-1:0] sampl2 [0:8];
reg [3:0] scnt [0:8];
wire [NVOICE:0] pe_accz;

always_comb
begin
	m_cyc_o = |(wave_request & wave_sel);
	m_stb_o = |(wave_request & wave_sel);
	wave_ack = {9{m_ack_i}} & wave_sel;
	m_we_o = wave_sel[8];
	m_sel_o = {MDW/8{1'b1}};
	m_adr_o = wave_addr[wave_senc];
	m_dat_o = sampl1[8];			// there is only 1 output
end

generate begin : gWtaOffs
	for (g = 0; g < NVOICE+1; g = g + 1) begin
edge_det uedacc (.rst(rst_i), .clk(clk_i), .ce(1'b1), .i(acc[g][31:20]==12'd0), .pe(pe_accz[g]), .ne(), .ee());


always_comb
if (g==8)
	wave_i[g] = aud_i;
else
	wave_i[g] = sampl2[g][15:0];

always_ff @(posedge clk_i)
if (rst_i) begin
	sampl1[g] <= {MDW{1'b0}};
	sampl2[g] <= {MDW{1'b0}};
end
else begin
	if (pe_accz[g] && scnt[g]==4'd0) begin
		if (g==8)
			sampl1[g] <= sampl2[g];
		else
			sampl2[g] <= sampl1[g];
	end
	else if (pe_accz[g]) begin
		sampl2[g] <= sampl2[g] >> 16;
		if (g==8)
			sampl2[g][MDW-1:MDW-16] <= wave_i[g];
		else
			sampl2[g][MDW-1:MDW-16] <= 16'd0;
	end
	if (wave_ack[g] && g != NVOICE)
		sampl1[g] <= m_dat_i;
		
end
always_ff @(posedge clk_i)
if (srst[g]|rst_i) begin
	scnt[g] <= 4'd0;
end
else begin
	if (pe_accz[g]) begin
		scnt[g] <= scnt[g] + 4'd1;
		if (scnt[g]==MDW/16-1)
			scnt[g] <= 4'd0;
	end
end

always_ff @(posedge clk_i)
begin
	// Writing the wave table address sets offset to zero.
	if (cs_i && adr_i[8:5]==g && adr_i[4:2]==3'b111 && vt[0])
		wta_offs[g] <= 32'd0;
	if (cs_i && adr_i[8:0]==9'h13C && we_i)
		playback_ended <= playback_ended & ~dat_i;
	case(wave_state[g])
	ST_WAIT:
		if (pe_accz[g] && scnt[g]==4'd0 && gate[g] && vt[0])
			wave_state[g] <= ST_FETCH;
	ST_FETCH:
		begin
			wave_request[g] <= 1'b1;
			wave_addr[g] <= wtadr[g] + wta_offs[g];
			wave_state[g] <= ST_ACK;
		end
	ST_ACK:
		if (wave_ack[g]) begin
			wave_request[g] <= 1'b0;
			wta_offs[g] <= wta_offs[g] + 32'd32;
			if (wta_offs[g]==relese[g]) begin
				wta_offs[g] <= 32'd0;
				playback_ended[g] <= 1'b1;
			end
			wave_state[g] <= ST_WAIT;
		end
	endcase
end
end
end
endgenerate


// note generator - multi-channel
generate begin : gToneGenerators
	for (g = 0; g < NVOICE; g = g + 1)
PSGToneGenerator u1
(
  .rst(rst_i),
  .clk(clk100_i),
  .ack(wave_ack[g]),
  .test(test[g]),
  .vt(vt[g]),
  .freq(freq[g]),
  .pw(pw[g]),
  .acc(acc[g]),
  .pch_i(tg_o[(g+NVOICE-1)%NVOICE]),
  .prev_acc(acc[(g+NVOICE-1)%NVOICE]),
  .wave(wave_i[g]),
  .sync(sync[g]),
  .ringmod(ringmod[g]),
  .fm_i(fm[g]),
  .o(tg_o[g])
);
end
endgenerate

generate begin : gEnvelopeGenerators
	for (g = 0; g < NVOICE; g = g + 1)
PSGEnvelopeGenerator u2
(
  .rst(rst_i),
  .srst(srst[g]),
  .clk(clk100_i),
  .gate(gate[g]),
  .attack(attack[g]),
  .decay(decay[g]),
  .sustain(sustain[g]),
  .relese(relese[g]),
  .o(env[g]),
  .envState(es[g])
);
end
endgenerate


// shape output according to envelope
generate begin : gShaper
	for (g = 0; g < NVOICE; g = g + 1)
PSGShaper u5
(
	.clk_i(clk100_i),
	.ce(1'b1),
	.tgi(tg_o[g]),
	.env(eg[g] ? env[g] : 8'hFF),
	.o(out[g])
);
end
endgenerate

// Sum the channels not going to the filter
generate begin : gSum
	for (g = 0; g < 4; g = g + 1)
always_ff @(posedge clk100_i)
sum[g] <= 
    {2'd0,(out[0] & {20{outctrl[0][g]}})} +
    {2'd0,(out[1] & {20{outctrl[1][g]}})} +
    {2'd0,(out[2] & {20{outctrl[2][g]}})} +
    {2'd0,(out[3] & {20{outctrl[3][g]}})} +
    {2'd0,(out[4] & {20{outctrl[4][g]}})} +
    {2'd0,(out[5] & {20{outctrl[5][g]}})} +
    {2'd0,(out[6] & {20{outctrl[6][g]}})} +
    {2'd0,(out[7] & {20{outctrl[7][g]}})}
    ;
end
endgenerate

// Sum the channels going to the filter
always_ff @(posedge clk100_i)
fsum <= 
    {3'd0,(out[0] & {20{filt[0]}})} +
    {3'd0,(out[1] & {20{filt[1]}})} +
    {3'd0,(out[2] & {20{filt[2]}})} +
    {3'd0,(out[3] & {20{filt[3]}})} + 
    {3'd0,(out[4] & {20{filt[4]}})} + 
    {3'd0,(out[5] & {20{filt[5]}})} + 
    {3'd0,(out[6] & {20{filt[6]}})} + 
    {3'd0,(out[7] & {20{filt[7]}})}
    ;

// The FIR filter
`ifdef BUS_WID8
PSGFilter38 u8
(
	.rst(rst_i),
	.clk(clk_i),
	.clk100(clk100_i),
	.wr(we_i && cs && adr_i[8:7]==2'b11),
  .adr(adr_i[6:0]),
  .din(dat_i),
  .i(fsum),
  .crd(crd),
  .o(filt_o)
);
`else
PSGFilter3 u8
(
	.rst(rst_i),
	.clk(clk_i),
  .clk100(clk100_i),
	.wr(we_i && cs && adr_i[8:7]==2'b11),
  .adr(adr_i[6:2]),
  .din({dat_i[15],dat_i[11:0]}),
  .i(fsum),
  .crd(crd),
  .o(filt_o)
);
`endif

wire [29:0] aout [0:3];

// Last stage:
// Adjust output according to master volume
generate begin : gAudout
	for (g = 0; g < 4; g = g + 1) begin
// Sum the filtered and unfiltered output
always @(posedge clk100_i)
	sum2[g] <= sum[g] + filt_osel[g] ? filt_o[38:17] : 20'd0;

PSGVolumeControl u10
(
	.rst_i(rst_i),
	.clk_i(clk100_i),
	.i(sum2[g]),
	.volume(volume),
	.o(aout[g])
);
always_ff @(posedge clk100_i)
	aud_o[g] = aout[g][29:12];
end
end
endgenerate


endmodule
