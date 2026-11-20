`default_nettype none

// Sine Lookup Table for 3D Rotation (0..359 degrees)
//
// Angle input : 9-bit unsigned degrees [0..359]
//               Values 360..511 wrap modulo 360 (so angle + 9'd90 works directly for cos).
// Output      : signed Q2.14 (16-bit two's complement), 1.0 = 16384
//               Matches platonic_lut.v coordinate and normal fixed-point format.
module sin_lut (
    input  wire [8:0]         angle,
    output reg  signed [15:0] sin_val
);

    localparam COORD_WIDTH = 16;
    localparam FRAC_BITS   = 14;

    // Wrap [360..511] into [0..151] so callers can compute cos(a) via sin(a + 9'd90)
    wire [8:0] angle_norm = (angle >= 9'd360) ? (angle - 9'd360) : angle;

    always @(*) begin
        case (angle_norm)
            9'd0  : sin_val = 16'sd0    ; // sin(  0 deg) = +0.000000
            9'd1  : sin_val = 16'sd286  ; // sin(  1 deg) = +0.017452
            9'd2  : sin_val = 16'sd572  ; // sin(  2 deg) = +0.034899
            9'd3  : sin_val = 16'sd857  ; // sin(  3 deg) = +0.052336
            9'd4  : sin_val = 16'sd1143 ; // sin(  4 deg) = +0.069756
            9'd5  : sin_val = 16'sd1428 ; // sin(  5 deg) = +0.087156
            9'd6  : sin_val = 16'sd1713 ; // sin(  6 deg) = +0.104528
            9'd7  : sin_val = 16'sd1997 ; // sin(  7 deg) = +0.121869
            9'd8  : sin_val = 16'sd2280 ; // sin(  8 deg) = +0.139173
            9'd9  : sin_val = 16'sd2563 ; // sin(  9 deg) = +0.156434
            9'd10 : sin_val = 16'sd2845 ; // sin( 10 deg) = +0.173648
            9'd11 : sin_val = 16'sd3126 ; // sin( 11 deg) = +0.190809
            9'd12 : sin_val = 16'sd3406 ; // sin( 12 deg) = +0.207912
            9'd13 : sin_val = 16'sd3686 ; // sin( 13 deg) = +0.224951
            9'd14 : sin_val = 16'sd3964 ; // sin( 14 deg) = +0.241922
            9'd15 : sin_val = 16'sd4240 ; // sin( 15 deg) = +0.258819
            9'd16 : sin_val = 16'sd4516 ; // sin( 16 deg) = +0.275637
            9'd17 : sin_val = 16'sd4790 ; // sin( 17 deg) = +0.292372
            9'd18 : sin_val = 16'sd5063 ; // sin( 18 deg) = +0.309017
            9'd19 : sin_val = 16'sd5334 ; // sin( 19 deg) = +0.325568
            9'd20 : sin_val = 16'sd5604 ; // sin( 20 deg) = +0.342020
            9'd21 : sin_val = 16'sd5872 ; // sin( 21 deg) = +0.358368
            9'd22 : sin_val = 16'sd6138 ; // sin( 22 deg) = +0.374607
            9'd23 : sin_val = 16'sd6402 ; // sin( 23 deg) = +0.390731
            9'd24 : sin_val = 16'sd6664 ; // sin( 24 deg) = +0.406737
            9'd25 : sin_val = 16'sd6924 ; // sin( 25 deg) = +0.422618
            9'd26 : sin_val = 16'sd7182 ; // sin( 26 deg) = +0.438371
            9'd27 : sin_val = 16'sd7438 ; // sin( 27 deg) = +0.453990
            9'd28 : sin_val = 16'sd7692 ; // sin( 28 deg) = +0.469472
            9'd29 : sin_val = 16'sd7943 ; // sin( 29 deg) = +0.484810
            9'd30 : sin_val = 16'sd8192 ; // sin( 30 deg) = +0.500000
            9'd31 : sin_val = 16'sd8438 ; // sin( 31 deg) = +0.515038
            9'd32 : sin_val = 16'sd8682 ; // sin( 32 deg) = +0.529919
            9'd33 : sin_val = 16'sd8923 ; // sin( 33 deg) = +0.544639
            9'd34 : sin_val = 16'sd9162 ; // sin( 34 deg) = +0.559193
            9'd35 : sin_val = 16'sd9397 ; // sin( 35 deg) = +0.573576
            9'd36 : sin_val = 16'sd9630 ; // sin( 36 deg) = +0.587785
            9'd37 : sin_val = 16'sd9860 ; // sin( 37 deg) = +0.601815
            9'd38 : sin_val = 16'sd10087; // sin( 38 deg) = +0.615661
            9'd39 : sin_val = 16'sd10311; // sin( 39 deg) = +0.629320
            9'd40 : sin_val = 16'sd10531; // sin( 40 deg) = +0.642788
            9'd41 : sin_val = 16'sd10749; // sin( 41 deg) = +0.656059
            9'd42 : sin_val = 16'sd10963; // sin( 42 deg) = +0.669131
            9'd43 : sin_val = 16'sd11174; // sin( 43 deg) = +0.681998
            9'd44 : sin_val = 16'sd11381; // sin( 44 deg) = +0.694658
            9'd45 : sin_val = 16'sd11585; // sin( 45 deg) = +0.707107
            9'd46 : sin_val = 16'sd11786; // sin( 46 deg) = +0.719340
            9'd47 : sin_val = 16'sd11982; // sin( 47 deg) = +0.731354
            9'd48 : sin_val = 16'sd12176; // sin( 48 deg) = +0.743145
            9'd49 : sin_val = 16'sd12365; // sin( 49 deg) = +0.754710
            9'd50 : sin_val = 16'sd12551; // sin( 50 deg) = +0.766044
            9'd51 : sin_val = 16'sd12733; // sin( 51 deg) = +0.777146
            9'd52 : sin_val = 16'sd12911; // sin( 52 deg) = +0.788011
            9'd53 : sin_val = 16'sd13085; // sin( 53 deg) = +0.798636
            9'd54 : sin_val = 16'sd13255; // sin( 54 deg) = +0.809017
            9'd55 : sin_val = 16'sd13421; // sin( 55 deg) = +0.819152
            9'd56 : sin_val = 16'sd13583; // sin( 56 deg) = +0.829038
            9'd57 : sin_val = 16'sd13741; // sin( 57 deg) = +0.838671
            9'd58 : sin_val = 16'sd13894; // sin( 58 deg) = +0.848048
            9'd59 : sin_val = 16'sd14044; // sin( 59 deg) = +0.857167
            9'd60 : sin_val = 16'sd14189; // sin( 60 deg) = +0.866025
            9'd61 : sin_val = 16'sd14330; // sin( 61 deg) = +0.874620
            9'd62 : sin_val = 16'sd14466; // sin( 62 deg) = +0.882948
            9'd63 : sin_val = 16'sd14598; // sin( 63 deg) = +0.891007
            9'd64 : sin_val = 16'sd14726; // sin( 64 deg) = +0.898794
            9'd65 : sin_val = 16'sd14849; // sin( 65 deg) = +0.906308
            9'd66 : sin_val = 16'sd14968; // sin( 66 deg) = +0.913545
            9'd67 : sin_val = 16'sd15082; // sin( 67 deg) = +0.920505
            9'd68 : sin_val = 16'sd15191; // sin( 68 deg) = +0.927184
            9'd69 : sin_val = 16'sd15296; // sin( 69 deg) = +0.933580
            9'd70 : sin_val = 16'sd15396; // sin( 70 deg) = +0.939693
            9'd71 : sin_val = 16'sd15491; // sin( 71 deg) = +0.945519
            9'd72 : sin_val = 16'sd15582; // sin( 72 deg) = +0.951057
            9'd73 : sin_val = 16'sd15668; // sin( 73 deg) = +0.956305
            9'd74 : sin_val = 16'sd15749; // sin( 74 deg) = +0.961262
            9'd75 : sin_val = 16'sd15826; // sin( 75 deg) = +0.965926
            9'd76 : sin_val = 16'sd15897; // sin( 76 deg) = +0.970296
            9'd77 : sin_val = 16'sd15964; // sin( 77 deg) = +0.974370
            9'd78 : sin_val = 16'sd16026; // sin( 78 deg) = +0.978148
            9'd79 : sin_val = 16'sd16083; // sin( 79 deg) = +0.981627
            9'd80 : sin_val = 16'sd16135; // sin( 80 deg) = +0.984808
            9'd81 : sin_val = 16'sd16182; // sin( 81 deg) = +0.987688
            9'd82 : sin_val = 16'sd16225; // sin( 82 deg) = +0.990268
            9'd83 : sin_val = 16'sd16262; // sin( 83 deg) = +0.992546
            9'd84 : sin_val = 16'sd16294; // sin( 84 deg) = +0.994522
            9'd85 : sin_val = 16'sd16322; // sin( 85 deg) = +0.996195
            9'd86 : sin_val = 16'sd16344; // sin( 86 deg) = +0.997564
            9'd87 : sin_val = 16'sd16362; // sin( 87 deg) = +0.998630
            9'd88 : sin_val = 16'sd16374; // sin( 88 deg) = +0.999391
            9'd89 : sin_val = 16'sd16382; // sin( 89 deg) = +0.999848
            9'd90 : sin_val = 16'sd16384; // sin( 90 deg) = +1.000000
            9'd91 : sin_val = 16'sd16382; // sin( 91 deg) = +0.999848
            9'd92 : sin_val = 16'sd16374; // sin( 92 deg) = +0.999391
            9'd93 : sin_val = 16'sd16362; // sin( 93 deg) = +0.998630
            9'd94 : sin_val = 16'sd16344; // sin( 94 deg) = +0.997564
            9'd95 : sin_val = 16'sd16322; // sin( 95 deg) = +0.996195
            9'd96 : sin_val = 16'sd16294; // sin( 96 deg) = +0.994522
            9'd97 : sin_val = 16'sd16262; // sin( 97 deg) = +0.992546
            9'd98 : sin_val = 16'sd16225; // sin( 98 deg) = +0.990268
            9'd99 : sin_val = 16'sd16182; // sin( 99 deg) = +0.987688
            9'd100: sin_val = 16'sd16135; // sin(100 deg) = +0.984808
            9'd101: sin_val = 16'sd16083; // sin(101 deg) = +0.981627
            9'd102: sin_val = 16'sd16026; // sin(102 deg) = +0.978148
            9'd103: sin_val = 16'sd15964; // sin(103 deg) = +0.974370
            9'd104: sin_val = 16'sd15897; // sin(104 deg) = +0.970296
            9'd105: sin_val = 16'sd15826; // sin(105 deg) = +0.965926
            9'd106: sin_val = 16'sd15749; // sin(106 deg) = +0.961262
            9'd107: sin_val = 16'sd15668; // sin(107 deg) = +0.956305
            9'd108: sin_val = 16'sd15582; // sin(108 deg) = +0.951057
            9'd109: sin_val = 16'sd15491; // sin(109 deg) = +0.945519
            9'd110: sin_val = 16'sd15396; // sin(110 deg) = +0.939693
            9'd111: sin_val = 16'sd15296; // sin(111 deg) = +0.933580
            9'd112: sin_val = 16'sd15191; // sin(112 deg) = +0.927184
            9'd113: sin_val = 16'sd15082; // sin(113 deg) = +0.920505
            9'd114: sin_val = 16'sd14968; // sin(114 deg) = +0.913545
            9'd115: sin_val = 16'sd14849; // sin(115 deg) = +0.906308
            9'd116: sin_val = 16'sd14726; // sin(116 deg) = +0.898794
            9'd117: sin_val = 16'sd14598; // sin(117 deg) = +0.891007
            9'd118: sin_val = 16'sd14466; // sin(118 deg) = +0.882948
            9'd119: sin_val = 16'sd14330; // sin(119 deg) = +0.874620
            9'd120: sin_val = 16'sd14189; // sin(120 deg) = +0.866025
            9'd121: sin_val = 16'sd14044; // sin(121 deg) = +0.857167
            9'd122: sin_val = 16'sd13894; // sin(122 deg) = +0.848048
            9'd123: sin_val = 16'sd13741; // sin(123 deg) = +0.838671
            9'd124: sin_val = 16'sd13583; // sin(124 deg) = +0.829038
            9'd125: sin_val = 16'sd13421; // sin(125 deg) = +0.819152
            9'd126: sin_val = 16'sd13255; // sin(126 deg) = +0.809017
            9'd127: sin_val = 16'sd13085; // sin(127 deg) = +0.798636
            9'd128: sin_val = 16'sd12911; // sin(128 deg) = +0.788011
            9'd129: sin_val = 16'sd12733; // sin(129 deg) = +0.777146
            9'd130: sin_val = 16'sd12551; // sin(130 deg) = +0.766044
            9'd131: sin_val = 16'sd12365; // sin(131 deg) = +0.754710
            9'd132: sin_val = 16'sd12176; // sin(132 deg) = +0.743145
            9'd133: sin_val = 16'sd11982; // sin(133 deg) = +0.731354
            9'd134: sin_val = 16'sd11786; // sin(134 deg) = +0.719340
            9'd135: sin_val = 16'sd11585; // sin(135 deg) = +0.707107
            9'd136: sin_val = 16'sd11381; // sin(136 deg) = +0.694658
            9'd137: sin_val = 16'sd11174; // sin(137 deg) = +0.681998
            9'd138: sin_val = 16'sd10963; // sin(138 deg) = +0.669131
            9'd139: sin_val = 16'sd10749; // sin(139 deg) = +0.656059
            9'd140: sin_val = 16'sd10531; // sin(140 deg) = +0.642788
            9'd141: sin_val = 16'sd10311; // sin(141 deg) = +0.629320
            9'd142: sin_val = 16'sd10087; // sin(142 deg) = +0.615661
            9'd143: sin_val = 16'sd9860 ; // sin(143 deg) = +0.601815
            9'd144: sin_val = 16'sd9630 ; // sin(144 deg) = +0.587785
            9'd145: sin_val = 16'sd9397 ; // sin(145 deg) = +0.573576
            9'd146: sin_val = 16'sd9162 ; // sin(146 deg) = +0.559193
            9'd147: sin_val = 16'sd8923 ; // sin(147 deg) = +0.544639
            9'd148: sin_val = 16'sd8682 ; // sin(148 deg) = +0.529919
            9'd149: sin_val = 16'sd8438 ; // sin(149 deg) = +0.515038
            9'd150: sin_val = 16'sd8192 ; // sin(150 deg) = +0.500000
            9'd151: sin_val = 16'sd7943 ; // sin(151 deg) = +0.484810
            9'd152: sin_val = 16'sd7692 ; // sin(152 deg) = +0.469472
            9'd153: sin_val = 16'sd7438 ; // sin(153 deg) = +0.453990
            9'd154: sin_val = 16'sd7182 ; // sin(154 deg) = +0.438371
            9'd155: sin_val = 16'sd6924 ; // sin(155 deg) = +0.422618
            9'd156: sin_val = 16'sd6664 ; // sin(156 deg) = +0.406737
            9'd157: sin_val = 16'sd6402 ; // sin(157 deg) = +0.390731
            9'd158: sin_val = 16'sd6138 ; // sin(158 deg) = +0.374607
            9'd159: sin_val = 16'sd5872 ; // sin(159 deg) = +0.358368
            9'd160: sin_val = 16'sd5604 ; // sin(160 deg) = +0.342020
            9'd161: sin_val = 16'sd5334 ; // sin(161 deg) = +0.325568
            9'd162: sin_val = 16'sd5063 ; // sin(162 deg) = +0.309017
            9'd163: sin_val = 16'sd4790 ; // sin(163 deg) = +0.292372
            9'd164: sin_val = 16'sd4516 ; // sin(164 deg) = +0.275637
            9'd165: sin_val = 16'sd4240 ; // sin(165 deg) = +0.258819
            9'd166: sin_val = 16'sd3964 ; // sin(166 deg) = +0.241922
            9'd167: sin_val = 16'sd3686 ; // sin(167 deg) = +0.224951
            9'd168: sin_val = 16'sd3406 ; // sin(168 deg) = +0.207912
            9'd169: sin_val = 16'sd3126 ; // sin(169 deg) = +0.190809
            9'd170: sin_val = 16'sd2845 ; // sin(170 deg) = +0.173648
            9'd171: sin_val = 16'sd2563 ; // sin(171 deg) = +0.156434
            9'd172: sin_val = 16'sd2280 ; // sin(172 deg) = +0.139173
            9'd173: sin_val = 16'sd1997 ; // sin(173 deg) = +0.121869
            9'd174: sin_val = 16'sd1713 ; // sin(174 deg) = +0.104528
            9'd175: sin_val = 16'sd1428 ; // sin(175 deg) = +0.087156
            9'd176: sin_val = 16'sd1143 ; // sin(176 deg) = +0.069756
            9'd177: sin_val = 16'sd857  ; // sin(177 deg) = +0.052336
            9'd178: sin_val = 16'sd572  ; // sin(178 deg) = +0.034899
            9'd179: sin_val = 16'sd286  ; // sin(179 deg) = +0.017452
            9'd180: sin_val = 16'sd0    ; // sin(180 deg) = +0.000000
            9'd181: sin_val = -16'sd286 ; // sin(181 deg) = -0.017452
            9'd182: sin_val = -16'sd572 ; // sin(182 deg) = -0.034899
            9'd183: sin_val = -16'sd857 ; // sin(183 deg) = -0.052336
            9'd184: sin_val = -16'sd1143; // sin(184 deg) = -0.069756
            9'd185: sin_val = -16'sd1428; // sin(185 deg) = -0.087156
            9'd186: sin_val = -16'sd1713; // sin(186 deg) = -0.104528
            9'd187: sin_val = -16'sd1997; // sin(187 deg) = -0.121869
            9'd188: sin_val = -16'sd2280; // sin(188 deg) = -0.139173
            9'd189: sin_val = -16'sd2563; // sin(189 deg) = -0.156434
            9'd190: sin_val = -16'sd2845; // sin(190 deg) = -0.173648
            9'd191: sin_val = -16'sd3126; // sin(191 deg) = -0.190809
            9'd192: sin_val = -16'sd3406; // sin(192 deg) = -0.207912
            9'd193: sin_val = -16'sd3686; // sin(193 deg) = -0.224951
            9'd194: sin_val = -16'sd3964; // sin(194 deg) = -0.241922
            9'd195: sin_val = -16'sd4240; // sin(195 deg) = -0.258819
            9'd196: sin_val = -16'sd4516; // sin(196 deg) = -0.275637
            9'd197: sin_val = -16'sd4790; // sin(197 deg) = -0.292372
            9'd198: sin_val = -16'sd5063; // sin(198 deg) = -0.309017
            9'd199: sin_val = -16'sd5334; // sin(199 deg) = -0.325568
            9'd200: sin_val = -16'sd5604; // sin(200 deg) = -0.342020
            9'd201: sin_val = -16'sd5872; // sin(201 deg) = -0.358368
            9'd202: sin_val = -16'sd6138; // sin(202 deg) = -0.374607
            9'd203: sin_val = -16'sd6402; // sin(203 deg) = -0.390731
            9'd204: sin_val = -16'sd6664; // sin(204 deg) = -0.406737
            9'd205: sin_val = -16'sd6924; // sin(205 deg) = -0.422618
            9'd206: sin_val = -16'sd7182; // sin(206 deg) = -0.438371
            9'd207: sin_val = -16'sd7438; // sin(207 deg) = -0.453990
            9'd208: sin_val = -16'sd7692; // sin(208 deg) = -0.469472
            9'd209: sin_val = -16'sd7943; // sin(209 deg) = -0.484810
            9'd210: sin_val = -16'sd8192; // sin(210 deg) = -0.500000
            9'd211: sin_val = -16'sd8438; // sin(211 deg) = -0.515038
            9'd212: sin_val = -16'sd8682; // sin(212 deg) = -0.529919
            9'd213: sin_val = -16'sd8923; // sin(213 deg) = -0.544639
            9'd214: sin_val = -16'sd9162; // sin(214 deg) = -0.559193
            9'd215: sin_val = -16'sd9397; // sin(215 deg) = -0.573576
            9'd216: sin_val = -16'sd9630; // sin(216 deg) = -0.587785
            9'd217: sin_val = -16'sd9860; // sin(217 deg) = -0.601815
            9'd218: sin_val = -16'sd10087; // sin(218 deg) = -0.615661
            9'd219: sin_val = -16'sd10311; // sin(219 deg) = -0.629320
            9'd220: sin_val = -16'sd10531; // sin(220 deg) = -0.642788
            9'd221: sin_val = -16'sd10749; // sin(221 deg) = -0.656059
            9'd222: sin_val = -16'sd10963; // sin(222 deg) = -0.669131
            9'd223: sin_val = -16'sd11174; // sin(223 deg) = -0.681998
            9'd224: sin_val = -16'sd11381; // sin(224 deg) = -0.694658
            9'd225: sin_val = -16'sd11585; // sin(225 deg) = -0.707107
            9'd226: sin_val = -16'sd11786; // sin(226 deg) = -0.719340
            9'd227: sin_val = -16'sd11982; // sin(227 deg) = -0.731354
            9'd228: sin_val = -16'sd12176; // sin(228 deg) = -0.743145
            9'd229: sin_val = -16'sd12365; // sin(229 deg) = -0.754710
            9'd230: sin_val = -16'sd12551; // sin(230 deg) = -0.766044
            9'd231: sin_val = -16'sd12733; // sin(231 deg) = -0.777146
            9'd232: sin_val = -16'sd12911; // sin(232 deg) = -0.788011
            9'd233: sin_val = -16'sd13085; // sin(233 deg) = -0.798636
            9'd234: sin_val = -16'sd13255; // sin(234 deg) = -0.809017
            9'd235: sin_val = -16'sd13421; // sin(235 deg) = -0.819152
            9'd236: sin_val = -16'sd13583; // sin(236 deg) = -0.829038
            9'd237: sin_val = -16'sd13741; // sin(237 deg) = -0.838671
            9'd238: sin_val = -16'sd13894; // sin(238 deg) = -0.848048
            9'd239: sin_val = -16'sd14044; // sin(239 deg) = -0.857167
            9'd240: sin_val = -16'sd14189; // sin(240 deg) = -0.866025
            9'd241: sin_val = -16'sd14330; // sin(241 deg) = -0.874620
            9'd242: sin_val = -16'sd14466; // sin(242 deg) = -0.882948
            9'd243: sin_val = -16'sd14598; // sin(243 deg) = -0.891007
            9'd244: sin_val = -16'sd14726; // sin(244 deg) = -0.898794
            9'd245: sin_val = -16'sd14849; // sin(245 deg) = -0.906308
            9'd246: sin_val = -16'sd14968; // sin(246 deg) = -0.913545
            9'd247: sin_val = -16'sd15082; // sin(247 deg) = -0.920505
            9'd248: sin_val = -16'sd15191; // sin(248 deg) = -0.927184
            9'd249: sin_val = -16'sd15296; // sin(249 deg) = -0.933580
            9'd250: sin_val = -16'sd15396; // sin(250 deg) = -0.939693
            9'd251: sin_val = -16'sd15491; // sin(251 deg) = -0.945519
            9'd252: sin_val = -16'sd15582; // sin(252 deg) = -0.951057
            9'd253: sin_val = -16'sd15668; // sin(253 deg) = -0.956305
            9'd254: sin_val = -16'sd15749; // sin(254 deg) = -0.961262
            9'd255: sin_val = -16'sd15826; // sin(255 deg) = -0.965926
            9'd256: sin_val = -16'sd15897; // sin(256 deg) = -0.970296
            9'd257: sin_val = -16'sd15964; // sin(257 deg) = -0.974370
            9'd258: sin_val = -16'sd16026; // sin(258 deg) = -0.978148
            9'd259: sin_val = -16'sd16083; // sin(259 deg) = -0.981627
            9'd260: sin_val = -16'sd16135; // sin(260 deg) = -0.984808
            9'd261: sin_val = -16'sd16182; // sin(261 deg) = -0.987688
            9'd262: sin_val = -16'sd16225; // sin(262 deg) = -0.990268
            9'd263: sin_val = -16'sd16262; // sin(263 deg) = -0.992546
            9'd264: sin_val = -16'sd16294; // sin(264 deg) = -0.994522
            9'd265: sin_val = -16'sd16322; // sin(265 deg) = -0.996195
            9'd266: sin_val = -16'sd16344; // sin(266 deg) = -0.997564
            9'd267: sin_val = -16'sd16362; // sin(267 deg) = -0.998630
            9'd268: sin_val = -16'sd16374; // sin(268 deg) = -0.999391
            9'd269: sin_val = -16'sd16382; // sin(269 deg) = -0.999848
            9'd270: sin_val = -16'sd16384; // sin(270 deg) = -1.000000
            9'd271: sin_val = -16'sd16382; // sin(271 deg) = -0.999848
            9'd272: sin_val = -16'sd16374; // sin(272 deg) = -0.999391
            9'd273: sin_val = -16'sd16362; // sin(273 deg) = -0.998630
            9'd274: sin_val = -16'sd16344; // sin(274 deg) = -0.997564
            9'd275: sin_val = -16'sd16322; // sin(275 deg) = -0.996195
            9'd276: sin_val = -16'sd16294; // sin(276 deg) = -0.994522
            9'd277: sin_val = -16'sd16262; // sin(277 deg) = -0.992546
            9'd278: sin_val = -16'sd16225; // sin(278 deg) = -0.990268
            9'd279: sin_val = -16'sd16182; // sin(279 deg) = -0.987688
            9'd280: sin_val = -16'sd16135; // sin(280 deg) = -0.984808
            9'd281: sin_val = -16'sd16083; // sin(281 deg) = -0.981627
            9'd282: sin_val = -16'sd16026; // sin(282 deg) = -0.978148
            9'd283: sin_val = -16'sd15964; // sin(283 deg) = -0.974370
            9'd284: sin_val = -16'sd15897; // sin(284 deg) = -0.970296
            9'd285: sin_val = -16'sd15826; // sin(285 deg) = -0.965926
            9'd286: sin_val = -16'sd15749; // sin(286 deg) = -0.961262
            9'd287: sin_val = -16'sd15668; // sin(287 deg) = -0.956305
            9'd288: sin_val = -16'sd15582; // sin(288 deg) = -0.951057
            9'd289: sin_val = -16'sd15491; // sin(289 deg) = -0.945519
            9'd290: sin_val = -16'sd15396; // sin(290 deg) = -0.939693
            9'd291: sin_val = -16'sd15296; // sin(291 deg) = -0.933580
            9'd292: sin_val = -16'sd15191; // sin(292 deg) = -0.927184
            9'd293: sin_val = -16'sd15082; // sin(293 deg) = -0.920505
            9'd294: sin_val = -16'sd14968; // sin(294 deg) = -0.913545
            9'd295: sin_val = -16'sd14849; // sin(295 deg) = -0.906308
            9'd296: sin_val = -16'sd14726; // sin(296 deg) = -0.898794
            9'd297: sin_val = -16'sd14598; // sin(297 deg) = -0.891007
            9'd298: sin_val = -16'sd14466; // sin(298 deg) = -0.882948
            9'd299: sin_val = -16'sd14330; // sin(299 deg) = -0.874620
            9'd300: sin_val = -16'sd14189; // sin(300 deg) = -0.866025
            9'd301: sin_val = -16'sd14044; // sin(301 deg) = -0.857167
            9'd302: sin_val = -16'sd13894; // sin(302 deg) = -0.848048
            9'd303: sin_val = -16'sd13741; // sin(303 deg) = -0.838671
            9'd304: sin_val = -16'sd13583; // sin(304 deg) = -0.829038
            9'd305: sin_val = -16'sd13421; // sin(305 deg) = -0.819152
            9'd306: sin_val = -16'sd13255; // sin(306 deg) = -0.809017
            9'd307: sin_val = -16'sd13085; // sin(307 deg) = -0.798636
            9'd308: sin_val = -16'sd12911; // sin(308 deg) = -0.788011
            9'd309: sin_val = -16'sd12733; // sin(309 deg) = -0.777146
            9'd310: sin_val = -16'sd12551; // sin(310 deg) = -0.766044
            9'd311: sin_val = -16'sd12365; // sin(311 deg) = -0.754710
            9'd312: sin_val = -16'sd12176; // sin(312 deg) = -0.743145
            9'd313: sin_val = -16'sd11982; // sin(313 deg) = -0.731354
            9'd314: sin_val = -16'sd11786; // sin(314 deg) = -0.719340
            9'd315: sin_val = -16'sd11585; // sin(315 deg) = -0.707107
            9'd316: sin_val = -16'sd11381; // sin(316 deg) = -0.694658
            9'd317: sin_val = -16'sd11174; // sin(317 deg) = -0.681998
            9'd318: sin_val = -16'sd10963; // sin(318 deg) = -0.669131
            9'd319: sin_val = -16'sd10749; // sin(319 deg) = -0.656059
            9'd320: sin_val = -16'sd10531; // sin(320 deg) = -0.642788
            9'd321: sin_val = -16'sd10311; // sin(321 deg) = -0.629320
            9'd322: sin_val = -16'sd10087; // sin(322 deg) = -0.615661
            9'd323: sin_val = -16'sd9860; // sin(323 deg) = -0.601815
            9'd324: sin_val = -16'sd9630; // sin(324 deg) = -0.587785
            9'd325: sin_val = -16'sd9397; // sin(325 deg) = -0.573576
            9'd326: sin_val = -16'sd9162; // sin(326 deg) = -0.559193
            9'd327: sin_val = -16'sd8923; // sin(327 deg) = -0.544639
            9'd328: sin_val = -16'sd8682; // sin(328 deg) = -0.529919
            9'd329: sin_val = -16'sd8438; // sin(329 deg) = -0.515038
            9'd330: sin_val = -16'sd8192; // sin(330 deg) = -0.500000
            9'd331: sin_val = -16'sd7943; // sin(331 deg) = -0.484810
            9'd332: sin_val = -16'sd7692; // sin(332 deg) = -0.469472
            9'd333: sin_val = -16'sd7438; // sin(333 deg) = -0.453990
            9'd334: sin_val = -16'sd7182; // sin(334 deg) = -0.438371
            9'd335: sin_val = -16'sd6924; // sin(335 deg) = -0.422618
            9'd336: sin_val = -16'sd6664; // sin(336 deg) = -0.406737
            9'd337: sin_val = -16'sd6402; // sin(337 deg) = -0.390731
            9'd338: sin_val = -16'sd6138; // sin(338 deg) = -0.374607
            9'd339: sin_val = -16'sd5872; // sin(339 deg) = -0.358368
            9'd340: sin_val = -16'sd5604; // sin(340 deg) = -0.342020
            9'd341: sin_val = -16'sd5334; // sin(341 deg) = -0.325568
            9'd342: sin_val = -16'sd5063; // sin(342 deg) = -0.309017
            9'd343: sin_val = -16'sd4790; // sin(343 deg) = -0.292372
            9'd344: sin_val = -16'sd4516; // sin(344 deg) = -0.275637
            9'd345: sin_val = -16'sd4240; // sin(345 deg) = -0.258819
            9'd346: sin_val = -16'sd3964; // sin(346 deg) = -0.241922
            9'd347: sin_val = -16'sd3686; // sin(347 deg) = -0.224951
            9'd348: sin_val = -16'sd3406; // sin(348 deg) = -0.207912
            9'd349: sin_val = -16'sd3126; // sin(349 deg) = -0.190809
            9'd350: sin_val = -16'sd2845; // sin(350 deg) = -0.173648
            9'd351: sin_val = -16'sd2563; // sin(351 deg) = -0.156434
            9'd352: sin_val = -16'sd2280; // sin(352 deg) = -0.139173
            9'd353: sin_val = -16'sd1997; // sin(353 deg) = -0.121869
            9'd354: sin_val = -16'sd1713; // sin(354 deg) = -0.104528
            9'd355: sin_val = -16'sd1428; // sin(355 deg) = -0.087156
            9'd356: sin_val = -16'sd1143; // sin(356 deg) = -0.069756
            9'd357: sin_val = -16'sd857 ; // sin(357 deg) = -0.052336
            9'd358: sin_val = -16'sd572 ; // sin(358 deg) = -0.034899
            9'd359: sin_val = -16'sd286 ; // sin(359 deg) = -0.017452
            default: sin_val = 16'sd0;
        endcase
    end

endmodule

`default_nettype wire
