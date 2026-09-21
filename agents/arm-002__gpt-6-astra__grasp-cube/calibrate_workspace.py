#!/usr/bin/env python3
"""Standalone top-camera calibration, recorded placement and tabletop map.

Run with no arguments. On Windows it runs on protopi5 and retrieves the results.
On the Pi it runs locally. --check reads hardware without moving the arm.
Runtime outputs go to ../calibration_runs, outside this scripts folder.
"""
from __future__ import annotations

import argparse
import base64
import zlib
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = (*ARM_JOINTS, "gripper")
HOME_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
CLOSED_GRIPPER = 0.0  # Fully closed calibrated target, verified on this arm.
# The three recorded poses: calibrated hardware degrees, gripper 0..100.
POSES = [dict(zip(JOINTS, values)) for values in (
    (68.96703296703296, 13.89010989010989, 92.87912087912088, -85.71428571428571, 63.42857142857143, CLOSED_GRIPPER),
    (13.054945054945055, -92.65934065934066, 97.0989010989011, 25.23076923076923, 55.86813186813187, CLOSED_GRIPPER),
    (-38.72527472527472, -95.12087912087912, 97.18681318681318, 32.527472527472526, 55.86813186813187, CLOSED_GRIPPER),
)]
HERE = Path(__file__).resolve().parent
UNITS = {"arm": "degrees", "gripper": "percent"}
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61036318-if00"
CAMERAS = {
    "top": "/dev/v4l/by-id/usb-HHWei_Technology_Co.__Ltd._USB_Camera_HHW001-video-index0",
}

PLACEMENT_B85 = (
    'c-ri}-LhuKZKioE3Z0HEX9CCsQipvDJ##cM!JsADW``_lNZLI;9lrO$&nmgl?v-`)A$lSpTcskox%ao$Q<;f;0lfI{zy0M8|MbH@'
    'fB)UT{_yiJKmPRR-~N97&2NAGZ-4pWx4-}GfBEUhKmYpOuRnkPUw-(*uRs0#-+ucyzx~5cKmGj2AOHOQuRnbE%U{0#gMHPXe)##P'
    'zyImi@BZ-ppML!NpY@;oQNQkA{`}*wzx?+1|NXb$|NPJTHGll!AAkPghhOv`{Nv9*{^c(}{H%XZfBnM`fByBi|MoY}5Bl@>|Eyo~'
    '`~UN|zx>lr|MI6l{_yj6fBF8;`mg-;FaPx8Km4kH{llOB{-^)?-9P;4hyO?a;=lg<<1fGFzxes5KmF;epY(tI&2RtthhM+{<M+RQ'
    'Zy)G+w!i+ZzxkiNfd79!=FdO;-~aN%FZ#*8v(NY4KmEVI{rwuh8~Uw({r+eB+u#5C-M{?$2mMemVE^qne>>%WAM|@#zbE`1t+n?0'
    '-^Td;82Wwx^3%`1>R0~VAAk7Qzx(BfpMU)RPk;CK|Lt#o{NvyK%P)WbxAS}1-~Au|`9FR4pMU)GKmO^5@BXuY!FTIF?KyDcl)wG)'
    'AN;#~d0YNs|9gI9{p%n9=&$>aKmYizKYaJkKmAc(w0_)w`sttb+kW@cKm5ZlKj>%BPf7YkKY#z{f7CDfQUAp=$6x>Y`~T;+zek+&'
    '-~Pup_<Vc#*0-PQVEXQIR{riF_T9hY@qBCi=C_{n<ZnIxhYvpdgMUT$`PTI9mojAMmm>5(z7+bm^oQm7-s@Z68(7kJ&YzjaS^xIc'
    'pYZsm``aJ*Oitf+>Yu{&ona&Xn|^EEYxEEDcR&33`@jE_F0=pe=Vtr|)Fa~=`IV0R7RFZ}e%O!u(LSjDS=Zm_djHLjzx?LcfBNA!'
    '|NP@0|6&XFH$VNuZ+`gx5C8O=zyFtCe)IDW-~Um6`IG*M{?{LV^XpIg`#<<r@sGdpMP|$Vm+$}iFMskS{@2|m{+6zXAOHNv=M%*b'
    ')8GDXj3I=`Naq}4{WD{KX!Hkt_s4&UrSs+M=3cC?QU8TO2baB1-&211kM9V6C;GJnfBp48ytBVO^t-Y34?jJ>a(;=2e>Y{z#4o?*'
    'mx}N^9tdYYzvg$m*6-54%U`{7{@qx*5GEepKm*Ue*`I>{`rTiC`sGJ^+W*<pOv3;00CU)9`uZc=a>}0)7R>y~U<-crVI~gyGxthr'
    'CQ>j&HXcw=XDt}}4V3;v9HgA|<?jirL46t&Bv7QFNDe9|VMN~=UqJ1(r|}%Ro<@WEI4De@_7GkKzW42AKkS9m)_-hYboAJ;PuZA0'
    'i-{5@QA}rQAf^ELjMoEdF#w2R;>G%(=JaV!>uXLL8i?p2<p-4Te&@9HmHK})y6s)>^O$zRq=<Oa_TveLEt9w9G$%SJ)VKqoIenTF'
    'Cr)Ii)1kdA;$Hk7?(rJnr_){wS9AI}CrO+%ia10l#!2tR@XARc&S9%gZ%^2#)5kHLgejtkGyDQO*ZXSHtq$yqCLE0?8q>#bxk-=!'
    'iz4jcS^oLuvhC?V_H9pt2$2pWJ`XDC5ny_Reije+P=URIl7WjuwOFS)eV$X2Bf#Yd(0E)=9$i%LYQ&a79u=N-HuN?3^PrL(0TxK;'
    '{Z;Jn(NOLp8zGP{DAeE(ut9wsR1zfoycz7ekzS=P7bv|qu|*OG5l@E_pXZbW3A9jxAFV-mHoWY7KoT(@PVSxW^PrL>fhI|4&=(S`'
    'e)jiJf_6v}Tb~oCL46!l(j?Ggi9yE^EV^)-YiwIk2f;(b=Ja7sNs~a=BtXbCiE~SAU?}F6uxd@Bf>RPD&>{)FLb-egxG$Go-SQ%}'
    'u{DryP@hlfCRKtgl+ZV2S#^DEe2>iq_wkf2X)qzDL46!lQYFY#31oBxM<4qFY6hqy%ClG0=RqY|f-IEKFd-zoIWM3#PcJ)S{efN0'
    'QUEH+5@fN&)TQLtqkYL2?!R8PrbZPF>hqwIETK18(~oF-a6ZN*#UbnqY7V5eo72ZRC0Sy*EMcG7?y>ToFBF)QqnKjhZ29Q(oRTWB'
    'z&5_UqmuHH*JVDw=8m3IjIADho>LMfmPHZ=*RabjdinVOoRgklGQ8Rnbw%~Q_jyi9lvsP50dvP%p-UV=Ph^g;ru4q|c}_``(9eB<'
    '$F#_t_xp0%b4F3Dt|#cK*n;BaBuVU{m^9>e?TE&^#s~cGRk2-dS%4`C65FB(`<|98TyznR^wQ+jNqaMOcTEAOq(^9&Cui;LrC%iT'
    'q8o`v?q}~MtCR1a=al4#ZE^(fjedc>q@b7|A$I8w$ek+uJg1~aY*Qm}`ROG5zU}GRwQS8zU4W;{EekLuF+$@!u%+UO4dS;;E&wk8'
    '-}m$y)w^B+rldtMSg?oN@3<GO9A&Qf5*S^_Qa~vw5v*I-7=maB_N^io*4Pn45bTc0KhG&C5xmE-rcO^*2ws#3py@l~8K(>QDmW!2'
    'f-Q!Sy`yJCE*V0JmJM9`{2l6jzCSl95&E$;B)48puXTODI_dlL<uvsuThapT=Q$-Q!kTl>q5)pxPw)M?hn{UuKV(-%6ktk9gqRW`'
    'mO+mK%6Ck#JfC5ONiaGl|2(IpM2O`x^tf5kgC>`BJ2FS@-kPJMh|hCMN`&Yl8Kg&#briyDu($Qj6txp$vPT<@>BE?k5^*fb243PW'
    'AM;XMF3;(tAK<LMJbj*1k|K^p5gI>4d~k80(ih@YM)st$kEa4oNsKraMldZuo(8-suzOHongB>$zEZ#`i4jM`2(X{><diPP1dIIk'
    'v^SjY)hOVU#E4^I#6<R<8eH$9kv^JteGZ(vt5*t4sJ;>-=I>mr8}it(yi4U<KS)UBQ{PX!<`i(6i4)jz(#7QY!Rzrfo+8*Y2er8O'
    'c}|cxSx$o?jEgQNR7d2PP&*OeoW}HdOqej47(v#XYZ;AAH%`liT0S(nqw>#lqQuFf2pI0_Hs|}UHwRNVwg?lqRQ`EP>uXHY+NSMs'
    'yddwrxKcSSddec^8GU*BJg1#FnHb@flXiI;@*YL#N(##yKC3Z(9up@_7DVWqasu@3>9!|>xVEQvDCl6XfRiLnRu}<?qlFMIcU-f4'
    'v*T$4cZRS4(@B^th&WFNp{|toT@NGkr2pl<J5@L>CoKXZr4cRzcncz)yOTzS?dMj3DM=AvQpB2J@y`?9IiX&k!Vp`Zu(!R>b4pSK'
    'kSE;1EhA*TW77MTw+8?Sv|&U6rX)px1rdxKc5dfAh>-Iru*ALXm2gT@1XvUijcG@S-{(F7h25y$hV>NdDB|;+k`{q(;iibl<L&q3'
    'Y4F+*O8J_n*VE@YB`pFij1WDTOxlx66fq@CkIr_-x2E)AN=b^q$Y+cb8u&XU87`9#n}RYsi1<9Fq(q=qMJypt<MMlp#vUdaxm+PO'
    'F{L?uoKun_P<+#iSa0J^*?X$?_Tt@AM8PnjBt?(~5d$ZNEh69cz372Qn!W86U`kR1*}=rRsfA_vdq_S>_^*RNd)NCsrldp=xlSJT'
    'H#jb<>99Lai3aYgH@(keN=gKo60!H75cV#~!Qt(=Lg8=Ols-%;=@2A#za3}$YNB^tPk(MSXdKwPt(6NnB^_c}2(iUY<<0E%(n#|d'
    'gYDJrgB6&P46!VPI5CCw;02RDSCp>E?5SJ@l#&dwJc1B9KHc;E+Ju`X#8{Lq&}z}=F(nydd2_}(Uj2IBr$%m#A2_;b<|gP(@AI6J'
    '4B?)YbingTm((E5Au}jV?zuDtqjHl9v7@XKZP%NZ(bM`|(b|55akRH+0jDHGY_L#|qP2Zq%3py<VqT#>eW#oXI3*pzJo6O7w$}Cm'
    'bm63LG#nH0cvDn!`Y@*?L~PGy4DqUexYWN6c}BUr&0b^rIHsgTY*!-0hlIZy5NJ%!QKT$*I)*5Klw=6D7((=EpA`E0o`)DFM$4LM'
    'n$zbwB^|<Z>@48s0zS9tZ8;s}7%0hz-Re}pDJc=`F~pvpX@|?MhqQvFD9sMZKaVLX5$0J5Jm_g_za?-tN0!$nOSTg6c}_`*V2dI2'
    'Y;vbwIPYa(IM);EAvw?H^jS_PNfBb>GY0uIX`u_Ib)I382cY}f3os=mLR^WE5#lOdIMFkNOOG?^d`1DMq(q3j_s<yr_-+BaafDT`'
    '2IXgW6j6XF2@&R5InRoe(m9u451vt;VzuXu>EoD^5Mh>;rb1BYMeo5L!2oZeL8fj<E5MYbh~p!;#_{Hu>v94N)0H*t1vICRb4pUg'
    '@gQQkJC2NfOAkg9=vj8%YD^!;l#~by-SlC=G2N<P36MP?T)H>5Q*QyMBt^Jm<zny6(0@}doKE<=Da<+g?(}(1Ns2JXN&wt3>wCUO'
    '5h%LtxwLnv2Z9%HQX>Bj3f!|2X_G|316|7gZ3fhE??l{vKox+R36zcDqU~lK`r0mm1W#9%4%37lmsSA^5-5`+n8U5();@?<U*w*N'
    '%ctKpr%!Xj#L2T7`-p{na`8h%9*_0PI<ps41t?0OJeHu%Ui^PiBo=9;;q$umfa(#16`<DFpv<`v1dsPT$t5-)kC%~tE!anBPM_wq'
    '6DQXsbh&tM+Pie5p+;1~E?RT<##g|J6DNx#79K0zGx_d)LCFJ3NMAr-b_+mB0%f7ZVfU_-{XQ*nFI#&4MYwj?8&!Zh36zBrdp;-B'
    'x!j@7_8ak7cJ*yeALlfaDq-%GfDql&vfoqhizz@M3%9jL1*oJ-n17`Wl#V0nvhh8h@<-Ohj`#{NB~8NYE0NA}OqX5_Iv57bFuQ-c'
    'hEvicETanwApG`jSKTjHs3p4#%u7HeO~MQ;QS^x;ka*7*GKA5n#_t-L_Z_MLRMI5O!4iSNTQ)9kImA&WF@fo{t5F4~q)C{ACDKy@'
    '!51?(8etL!!)Apyr_Xarngn`wLm$f9s4pXp_kbN6pZ>Z!lqdj|GzoLDMC+EZP~!b~8jm6Do_6DEN}s2c1PSx71feK_nE0O6)y*#I'
    '(DifzY(?Vppdcv{W@3qRg%kGWqO0fA9?=4|R(HEsa7u!NxmbcbBbs^drS;P$YH;Ya%)4(;1)!27VLq0)^o`oa$CsA(wW5A`puL?2'
    'RRAh!66R!yxOY@jUb2Nd0`sZQH@f1of>V+t%*zrtPLJpIc;ER<Dc|AiM%ZN%6`+zNVP=+Oi9qu9Pr9hFvV2y+Lhfk|6`YbJVQ!Y7'
    '0Xl>y4qq13<MyGqEgw46pgs*MX%c2=iKx+@?R;}yPN*Gjii-rho~43Qk|fO0a)E3=p$J|zJ|4i}8y{JC*e6s0sH94m0U{DD^mO4R'
    '<ip2DLU&H+EK~s}OrnH&TH<)hBrZ<AOefl|%`mYKs0vO=lrT?AoMsB<k;FxmIAK~fne{1qK^1^XqJ)`R;s&|go-DBuV+(b~ofcvX'
    '5G6Unga}+4&=XLX5{a!V&4UOaJY8~Bz$wWQW@w3EN|5VD@0<i|x~KIa5<{C&Re(xzgqd3E>qM`Rpo^*H25au{oo_&!k5+(6dW5-J'
    '>b(fPr`l8MuS2Mx4p1}5ZcYW9k{n^KmVdR0@TSAJ1tsC-Ye78qz`F`iNsln271BqZG0pCBf6_R@7t}y$r}+v_NscgIOCo+6Krh9X'
    '`-H$}E+HhX8+{%WB|XB_1scmrD6OLCBA74o1U)QTyHkY)oRS`4&XyJ-_*%IbkjjX5wKMbqJM~_{DajG$Y`I95uNArV6+$@0+7wBc'
    'eTOOll|%{iwp_$9iW=!XrF&MCTWaLTyJPbTP)U_Acgtl_Ul`=Pb2=l2JctdhZcLxYlr#x*x7-#RL{s~Fq7a|uwDoJD1Bn7oNs>5T'
    '#WG{&DxXV6m!1g@fUtVoE8vtQ3A4A<{VWF8>|6fn%Uo6%0DXZf;FJ^zySTF1O@+dDSx@^>B<Rp=>8Ju!k|fOE@~^F(^L?76f&%Y7'
    '6!t2m2K8}J>nlkD-QQ9{!M9hGvUG37!E17ZDgZSTD9dh4bJ+9YHLskM<?zmTAeQ!>uiylUlWt?O4lHkky4YK43=!8XfI-r+c>yR)'
    'pti*k7z0`-U3NYuiTkki?DmQ(07VIu#uA(5_*M3jFJyfsUO{H^1xg2f1)$c~pv>P=)^-U`1TR4!={t{RU8XJ(E8w&fCkrJ;41^MT'
    '?*V0+G1G7?n*OO8)Tcpl0%frTPWGJ--b;@(IR7%tbqmj4Q3aqRf!Yqr%PEIf`oDmh>`m2r-QB#q{k;OzNuW%Z(8qCM;<BLT(+YLg'
    'I_*7QfN3XH!u%~oAK8mzF9(!{@3_9S7D96>;FLrOa8W{H=mzpFG#4Kvu2DR4t49@}k}6>amq)`u@b*rCOKTx~amuFC(<LiFB~=1k'
    'm7uun?#s@{h!P17y-M|hD&Ukv2{X9V2+lV(edzZTJdUtunc~82gZezEBubdUrA;FArnE~UYA*6Nn<)sRo!%-qB~bz)GKR;BnCE-3'
    'x$b)-#^#-6kJz9-4=RZg=5Q$%(D@aLFLzy_=&D{OCADr|04j+R=5VRoUMQ{abKjNIK2M8>1ryEb^PG|<fij0Vw?T}}ed8oZ3%no7'
    '`*vG9`aCC2qJ&vo>P>fg0T{fu*)ZXX;v+rB+B&fWR8l3(<MND+UUf;g6G{nrX4fpxSEt|$KqXazXbwH4<ixb<_e+%E#29rWmX2>w'
    'p9hsx3DSrgSzngLd>2azc$^c>K&(Ua3QS3qAd4gbyd6q8z4t>Mf@k7`Mu**CSOF@D5_%&VgkyPnUHHAjaDSQKWe#FJRlEXJQYG};'
    '4Ui~;7;E1=E;Z6~Jyv;kYjp*vq)J$YiiX|c?koGfbB%F(@>dt&Jl$bf0V=5y%R&j!*Y9EXC6?Ic+~)=%=+&9c>C>E&DzP+{&|49D'
    'e*N8qnRUZt4>;^2ajQq4=Ojs$*kb~2S)SRQuf3q4p&~>B$Hq3Ns=$;qiLHSI>eZFpmhUpiMpk%#Oy~iVC76;Tv8~Yc%y8{pmu#VJ'
    'cPF|xV0E^zfKw7AwnY&(G15*aT$km<^3+MpvrdN*1)!27p-(gP;NjzUD7e~@N>m;(_Qci_P)U+tJ(jkd&{_9>&%f^tf-$j`xm&9%'
    'KqX0nJ(5__z5y4F4>{};7wU1<ibMgYq)4!a5@g06zMwAsg}lBTQC5W1F<$|wq)70#m`^xH37uQI@I38Mvy+7tn35pD8cEE}Z03o='
    'Q{HRM@vrl{)@+5Z0Mto}go%4V`aMF5ec$*d3EHC5PcExn`6@sqO~QHxX;QS%0KRK{bkIQ!9%oSZffj&Ds)T4L0gDmB*W-QX+u=a3'
    'JD>Oru;%n>PDzyzjU)hvc;OMgr*-x8dO|+d%Wh|N3pgcFLaa`LHo-Ttx@K4^Uxw?h=2XBbX%feza=k!(X`s279?cjLzo!1cj?D`|'
    'B~9YMbW_-<xO~U-W!uvqkFzlqU`mq2ArJUsLJE%CfXTHZy<tD0(;Ee#k|c2~kYIKrH^B{;74_tyNN>hHxv&CM5+#nOH)2+`F?d&F'
    '=bRkb_p_z*P#*`SvE9FeqThMeeLsez7l+?N3;YVTG??%4g%zA;;-rzpT5w|U)1}@LC?+~Av&`;LqJR@5P8v$^f|&JrH7bDZJvOq*'
    'hNLv8&x67Q%0daz)5rbx<~t~yK;dQpQJC4FJ`IWzDDevB<u!WqTna9~WTEzw)9H-@PU~w<(;^8QB(&TZ@uKm4u@D;vpf|n(PCIe3'
    ')(zh1{=OGK3D7T!sE`>uGOysoiIa&Egub3G$CEW|Kr~fLH@{a<l0<2HBL;tQF?w4}9F&AK>T4IB3QQ+qvdo4EQ@AU?b6N;7epe%o'
    '_TwwyG?N?www)~;Cq}lu?|Ng!4GTeIz2;QFDajF_+?W<yEsNk~z~otRI<3?81QnE$7y;h=8A>=`Tu5n$3#qxEQ>Qg5I3+Ctww1)v'
    '@#;K$56Nw3jN^$HwIfvprzAyyWi^nG3*ndQh$-P_K3GQk993XSN(5REF>EHD8!KPR!bTLaz>-e8mvBl-1X{<2UWe@NG~seQ4T*`a'
    'OY}>-W21mmQX){ph;7cwZWTM<b5CpSQ1&+N_Fk^wl(Y!6FhW253hTD_%hRE_<sNfq>GHh-PDzX~V@uK~0iuBNGHm0SyemM|wqXUF'
    'Ac+w)yxNFDX?Qn^MILTVFi1dpVqpcRBu0>l5jdAug~@q8pD1QHoOx!C!K=WO)ChC6To7(xXWy!ppX<}vu(?wj1)P!^VWyUBBhy^l'
    'yM%j$K3r{%vsQDe;FQz|bG6)qp923|doIR$B76TGmKShJa)g;$o-g)#4Ex=aWuG;l?9=In_X<!+k1$iqjn7cy1D6TacE2^v!iG?G'
    'yjQ>}=@I5>xoL{qAYQC<<QXb?7Oc@5Uj?V6N2~}WtXuHNxw$S2$1si|tCP7EoRS{lmX>SHs5E-N=dQ;E$%Z_P9@1LC36mbNEijm<'
    '+nTMF%lJ<?gr5AcA!}`FSpg~u5@u>CdP;fQ=cOntiu(-aM(Dgo0j4BKn5iWTQCPd(>?6XDWeTm;mdh2Kk|1HGme%>i?h$(~=M!zW'
    'W5G2}=iMtfB|*YGE%h6CGvVY*#d6GUIJQIfNPr4VNsmwfa~|>f{~>g9Tz(8D=m7U}D&Uml2=*FFGqv<^?>+rQ7&F9iS^;x&`aGv3'
    'M<^p3Xy-Es=e+dOU|2`!U6@+ktH6}h2(z<PNZW#PxpcY<p}Q7XUGwUyunJB|jW9n;eevQpiQ}@Kc%P?vSKLjpIenNDB{f34RpY?$'
    'sxZ9Sjt-B?<@B8Q5==>pFf+?#l@IPZdch=7#G=vOJ{^=7U`kqqSy^tIO=zbR+!_hLoKB*PuA_(oPDzUpAJcVW;^%D|z(Ee5UmpGg'
    'o71N`B`w0NEH%;-hj6iB!Hr=X!DjB!>#2ZK5+lsYlK0@S-h0<zmy%rJ8lkVZ*HZzfBu1E%B~OIuhtFjq_1xj-W~W*2dlj6L7-3eH'
    'qN~Jn?(YVP+f<`y%pEejw7dXQ5+lsWaubE6-o@QRUnuXWv7D_%RA5S4gxOf~Wbf3Nc$4-*DqmCd2R5dUV_IKH5i3kA&xUB`d`}SS'
    '3y<;cOU{tiA__Ro#K{XR&+@?<dtUn7&0kr6M>gQSx4i;RkT{tbaS;7<lAAT+=2&zKsT1xMoG@|HC}M4Ek9K|aeLe9b!GcitPgh`~'
    'gvp|aW3I3sBD@zy(0cp`5V-rND>$vMIhljy+JijvbP4z5k(G37l%D!jfoUg9HlEAoa@nqTxjP9ij`@)?c(=4yz=;zlx57pU>yAsC'
    'J3Z66Y%F1?o+>y=;^b`_IDOl@Oi@Fh8)=cdFt<xs1*elZSz*M9Ne$z|i4~%oW$!b4h*|-rox}*Yur&GGGf$T&V$*a5_6(k#(Wu~*'
    '#0a;rv<7NlPnS4?;HiUMWG|>eeI8U&Bh1B;Y{y%-ORh|fH^PNw>NU496<|tYgqc|G6Z6r8FUtu-G_n~f?Sx%{DTxv0VafWz{bnSW'
    '*~TO4BIX1)rf*E2#+0-O^pxKM@1@!ADUG9VG=lQ6PHPbbn35J@4mgAo&b{(3?deLJGaQyrTYHpnN?L?jSfZepMfm%F3nG?Hj`SeH'
    '+?Jcu=Q$-U!YnKg9zUE8Uo2SgJY(*FPj|Ria7tQ)Sy=A9eFF8o+@1E0rY8}WwK|jgc}|?f2=bK1UY;R*Z%z|yUZbcgC+zW_6`Yb7'
    ';TD!Q<j&n*FZEzsVgi63PK$ay6>v&g1X-^qnjx=g<#MsY#>Z<Eu^l5SFeNR*EG&_2ewy_>UBW$q9ort-pz!AOc}_`;Ag`}n1Hu~d'
    'J&YiHCID>QT@Y5mDX9@|VF??-9a!i#<}Rjgm@%UCdMe<Q<Op-HTu8jX(K~Gm$vG*n-OH%}Q&J<AJ7Eh%p%JYMry1Qe`UZAemREpE'
    'a)dcp(p-@uBz(U<p>B?0H9EbpxjB8BQ<5Xh!IBBUoe}X>Ef-pkgC(9ea;v~3NsZX%Jf;8j<7vM;gu(n0?oEBfUQPv^k{YpL>PLZS'
    'xaCWZki}<9>SDsv1<Mtjk{V$KmU?sLsN8$MK3NfFY<nnO5LQ4bi4kUBx%e#CgD!;;`a-}9jflh@&zsYyIVCZ|>?;=&#U=NZlRn<l'
    'O*1rthmIl&I3+QHhr25~C$}Mg+w~Z{_FN2CZ+iupk`}?PMW9a_oVWF)553UKC(Kr-_q_s8NsVB)p4#GK`TKZJ8O#DcNqp1B?$apX'
    'l*9-#uv|i35;o3$=Ohcfdm8tgr`J;frzA$0fhBJ_)*~qMeK40S+zZ?fR$#SSUI6MOIl>$)7ixH53+jE2kid-j)B5=Odit7Mz$wWQ'
    ';`2Q*W;wkFe0phm8lRZZ?OK}Cr#U4#!fIT3C<V9okXsQ;$a^Npb}?)PrzA+2hvgm`@A>?+dAaG**pd3s|7$b13Q$Rr5U;W{OL8xL'
    'xI}zWy?5XUcGYqPr=&=jiKP{$ct!H{sw=dkmrx(DZ%_rCk|bd!mfOa(`vG+c_{QTLu}xj?w0i-kq)3>F<(g9@g1Pr}?wRNDd^^x-'
    '!U9Z5kZ==A{V@^m!DZXy73b52-Q5}`ppqitHkKM1c+h>}q|tbEsqO8bvN3%Y6OOO+i0w9(w2rvIH!s`Xc&hg_@Wa)ajRH<HadIzg'
    'ncg7%?uBilUX+f>!fv6izyt}CM-e;5PrnzJYe)`YnDm2hY`wgK6DCexAMy0v$~&iRErc;Y&9WfP>GPZ@aWXZ6hlt;D-_;1#xX?MB'
    'y>NCfVF9Q0H7B#N+{f$>y{hOUmp?Kr>oux-2`f15#0k7XEt+cJeLXE*b#C#obUn3ESOq6eoV;7ZV0j7UdmQn_WqF{k$i6-maFWEy'
    ';t0JDoo_UKl5oTtlQewq^;E#=Bu=JASVN(!d+%z5%;^sFvJPNRjx69blN{kjmTR-sg`LZKT1&qo(u4qQI9kCe$q{xmt;NCM5|@+D'
    'E%e-zK7F501)P!|VLq08Ae!Cr{hoW46(-qoG7s6t^m$B4jsOoMtWnJ#;Br2#i81^{Ke>A#HK<R6N_vDjS#BG-<#GA@J$Kt+V>i3B'
    'kfZm#0!~SfFe^*OV-r@7Tm!zd$X67b>2e2r1)!23flnY|f;sag;DhlPUc%-skEr031PL><JVWMk`-s<ujRSi?)R%9UavRg9F(p01'
    'ye#*`h4(=KdVgA8fj$mRXg8J$PDzh2FUxh-j6C6`dT;L$Wnp@Qa@Upy^<hwu1PM2@T)-~GzW1qZz_>p#a(m`fP)c%ySy^rX8Qw<y'
    '9+odQ8zfUKP`|A2xdoh(96{Fh=`Nzy*>t*1?SCOqNsyo@vy3i)OZDESCr=1-O4qSea7ucFnOSa-9b+@_5=M{@6d$maY3mUMoRS{l'
    'W|r3UdC%t_IFb6$m=H(rdlj6L9N}h`g3A`vM7r&JGf+<G5hxXyk{V%AoLHyYy=wVPee4rzIb(bHR&Yvkgn3zNya!&jd<!F(ebSm9'
    'PeR!CMg^Rb9I?VaLQLKBFzzi)$YJxLEN6S~6=1?7Mr;=&#8sm+<+7gSQH{p(8DGum)0~nVu}zL(@XB$)OH^(R9UeuDv!ug_0!~Se'
    'FfU8%!U#^6>hh%#p>KQ4tM|PMPDzb0H_HtebEq#?%RGRh=M(iXYfc56k{V%dmiAxob-x`?#KSu}Xz$beUInM5M!1`$)x-KN_q8hQ'
    '=$CL@pIAC5FQAmP2s5)}*xPSLmmHx6c~J~&HDnuGG%5g<)Cea1*hn!Gcf5rWHtvp(+e<9;dvas?G^Qj*@SaoS>=RW){I0>mV)uO`'
    'VQx#e3QkFkFfYqP$9q1{CE6RLt1c|v#7%da(}y`xQX{Nle2*QL1ayhYCx#^fu#aMEOrOS-#0Yn?<jJA0%6qE@JT27vh`USoDmW!E'
    'LQISRc}&3GHCTaX+oPiks*k4vPDzY#C(ET5A}@`2FA1|JqX;8b8}F5HN@9c?Ssvjyp7xDXwC3tHw;KQ|FeNR*d@Mya$I)}qBF3Y^'
    'vgp${rvgq%jW8ceef3`UAct-h%M(4;u%fU&o(ec6F~WQ-HQF1l8cn%4<~)uBbO^fAsDe{cBhI#VPeJmN=XKj#Yf2~<_I^5*TYxEv'
    '5oTj4XD-+6E>bxlNOb1%faX-dX?-O|u=`kU*7ZWSwp?L|IwOr07xm#(KxrmQ;HwF|fW<3Q-U|0@Dxuqr9-@vSDnLO3Wd)XiF(YkV'
    'QcrrSdE`!d9qu5a023xmI|^9b&W+y-jgJ0ca3f}FWv_w~B}x`U%)PxVVxHTk7uCuO#eFB5)2BJDuQ^RGi&!zL;eE8R=s%AhchReE'
    'E3g%ucH#tXHhSV>bg5R}Bq15GzWbJp3OI4%WKzWDGs4X6T{1fOEGU1sy`Ty}NdmRqEyrt-Wyobgomf!w@ahAqg40QyERbNUMz$c~'
    'X2P<UYkO;rHp4E!w38NLCYDNx#aGJZlDlAgCie^&Jxi#9Q&J<qQyMZXLpUd1&L=NaHV1k+ZQfhKDTxu_-c#)Qs@{=qs*zuEk2TXk'
    'Piv?EmE;JsvE*f*<Q{ZiR3jL^IW9ij@m>X|BuIeI=AtNJp-a5C{yGI<k9Nx`z?Ad|`w3$*<Gpgv>Xc!-|HnRyTZJv)l=KMmuw)xb'
    '=*tN&+aAU^JREy=y>bDjBuAKqrN#vkF~YmIhsNbG@=2MxLfD)>%_*r7=3#lp@Nt9Z8kXBN%O$``dfRGHp9huX2s5$dWf@u@c6V7('
    'qA$ibzJ+Z*Rltdp9$_Aqte@FUVdY&m(p48SEUbmymRrFo=@I5)Y3&y)N>twCy>UX05P+7ADljEE!b~h>9h%>&ScbUe4%#M1G^fvV'
    'N@|38Sc-0Z&Or7%rx|I&$zqdIn$zbwB{jl4ECneVHP%HnvTN@7@}I`^c}z)-Sf)nEn$h?4zMOO^#iRyZj@<p86`Yb9u{^6m9NvhR'
    '(mgscqUP9`lonwNI3+p4JuHzH_~^)Y4@-Te?zuPTwhHIw^m$H6j#$>Hv2@?_zTEe4Z|g<aLyZa4{nI6!k{YqR%5pw_ua|Jo^jGI('
    'eNwGPe4djeHNs3Rc}w_+-jcb?c0B)`Lhbcbz$vK_W@4#90(cMmrSS>Acq0Ncwpw0*DX9^grd7jYOw7Hwge@38`^3s!R(GgXfJ$<N'
    'nOK_1if1hLdp394bS_UnneS-J5e1x*9I@RC`}n>mmu#*^d27bkz-~?zoRS{F!{P#sBj}pW?R~GFQ_>^M#?lJ=eXU#q361f#-=EH='
    'Hp4FAl=KL;KmyM6Swio*TzkIAPmRN~F%@7+as*o#!7E&0@!fvvki<Rrh)n70Qvs(WN3i=~6M0;IX(n`iFI+73fBSg)Jg1Z72y?MC'
    '`%pO7iSB3M4mn~Y&e27d1)P!`;Wn02fX3Z02abduvCWFU(;5Ywk{)3;mKw>$$S1lC0njkd^G}CaZ_0gtDgc%A2y0%SR`@`M;5AXG'
    'k2NJ`-C0txr)pGyN`izLS&D{x-llddkC3nm!@bt-yhZ`0q(>anBlaHCx3r5vBGWUAb(lXvgDL=(1PODp)KhA?5PL~LjT7rhSjZhH'
    'n$xE_B|+l&kWXNnK`j^C9KGR=`6P5l5BaR%loW|$U7rU;VoSSabf1k+x5?E5g(^TLNy5D>^~1#V=#qhw1rSOs(&`2EaZY-t{dZ2{'
    'UY5M@na1#nNuL;_;7CLH)p2<NrkOB-?|X-RirQtK&{^acU444s=2XB55-0E1*x_6xa<gOcaja}N-4<dCFk!;vVZ;tiNa&g;WH$>t'
    'qdApuqQuFf2zy_Cg}u5GJkyQ!x6p!l0jKpfCv&sZyHg|z$9p|2@R>7$av0s6Tfu23P9{g7fIF9ub8(oTVd`d%zpM4~0#2Mbk((R&'
    '(2V(>&^;QH$BaoZQ|vyU3OGsPw7m~N+=S|VsQZ~5^+waiAakn`1)NUe<ZYiYy-n@XMh#CLi!ve~nb({?%xNYy0wRt$;A5!Y>tT`P'
    'L=goG^nk7kPDze%H%rvjau4?i#-Q+Y6OTk1)2A^dH3D3X0E*dLm-6y6(amA^qS@J}3QkFmFfU8l9GqA#*C#!fJS04`#k~qlNsS0A'
    '%d1%qx=KDbZ|P{ffKO5(^d}4r2x2zc<<?}Q!7*lP=;_t#sDM*aA<V<l%3xiESnuj9#stu#*DKyV@u`ASQX$Z~G!7l_dg8Y-SY2&<'
    'N>s0`*^~+>B@H4hEcf!V6uL-y!X7(J5b8~@0#lM8%)s(!<mQWM?#rf!=yH+(P_L%XV}hhWxPRrg8fANccS`3Oa%Y}7*sG}kQxYJ^'
    'LkRH*LM~%ZNY0E=LvjH6W?R51DG=^oX;XxJ*Smyz>nSML;GH|+RKY1J5awTL5Zvk&F8khm673*7cOOp$oRS3L{*_}b@!FwxPRlF^'
    'TpvdmZ4g$$DM=B_q==yxSoaydDC9e)Jsr@iQztSiI3+1UznxDt#&0-J@q`h6uv>PA5e1l%65;-p9LLkSZF>tsAfJ0lkJG5&l(dLt'
    'VZ?cs6JLCL7d)k68lZO+QGh9F5oTX`=D_)lYH}2gU2h1U&fgYr!X!qReWhMty?O9l+Hq+=!)a(kH%(VyN@9e&SK544cY}RDodnj1'
    '^zE9ni(d;cB`so`Tf`Dzy4L*DdOWakg0z`Y1*RlLn0MvbMBaC?E>gsk(6GSpNCy!GoRSo=eLy&_b@gyN3t@OZg8+Lzcm<^-L~sZZ'
    ')0;QnBZw7WpvM>N*-6|AOi70@<I1BS+!qsE@)>7F5%XNqZMYSfk`BS9Ll7;$6kQ@ceJSh(l_(m+_r<AzQ_>-L>-u05%5T2^mcy8?'
    'L(F3y%#G>un34{`-a9j65Tv{p7;QbCyspyhaeCV;;6zD>FwdJ~W_zX=U37@07h?$*BfUfQ4Yz<(5+VeAKEv^A5%DFP@$_XJ#?d~o'
    '6`Yb1p-XAvmPl2cOJOC&HTbZ;n#NRsDJc=M&82)#w^gQjxja3lE3mlT2TC|4B|<|7>xfxCVer*!3iOpM48xDL+^4TOeVS9!BFwcC'
    '_09VY7G5#g3+EWD)&;N?kdhK{EQXj79(aY<tFAZP<jDLXI?^kkl!S=mlel!X*nYO&jXZ=-cv0!~R+m$A`aGv3L>vntFy?v6#Sd%`'
    'h8V<Um6eU@^O%wlam1FB4P`e>?|UA{xHOw%EWPP{p3?eChB$^`+Qlg>1ul8^$Gd1w5?;NU3OLQg$wCMuUSSyeE<>!SeU)u<(;Cy~'
    'F+sva)(dDO++BuvUrsSb(qyHzy1b}@6DCgE)z@glotL5Dn<@IC#^P4>DmYQ%WL1^2WLtgF1rtXKcd%ZvxVOCmOzUe*)1n9fw=SCe'
    'Ubu4Rlfykne_*Rc1)O%`q)`N{#rt0JfB3mM={<OLOkO~V6D2RI<heXG_`aHCMpfmo=j)KX0Fxw4rbFo4V!HeM?@i(b5qQkttvP+3'
    '(@C66h&ZvFZjNnxM673_Udyiqn08VkV3=NnVC|hZkz9XBtT@EcN<;~#q(p!hL~QmwZC_$NbLEW&njQ~Xf+;BxKpHv#AIG#ViNWv#'
    ')BW*uvvdijBt%$g_+;7N?C!+a^>itX+4T?=oRSa$Ru5tC%XhT*M7xIMeswx}9qxX#3QS3eu%7@{DDKIM%MBOAg$MyUCbT!EPh(0t'
    '1ey*pmWy`tCci%B@pI7{trt~rN=k&eRZgK8$o|@p2E(#b7)8o9F0DCzo>P({(4q(f6;mJIvxEA8k14r%kT$2!b4pSKis|GV-R_Gn'
    'X38)mYRS7bqk<AAC4x+epm2XU`&>%?Ey-@T;MM*`6_k<=L8e1Ur2WQwA-S9tTi!wLTt)?@Btwu!5XXiz(als;QJ!_ey@-28Pz9x='
    'LeO@J99;jtIJTYPE{RGCx}&`WQxYM_G8t_C$YywRNenLWkbUa4tA}Qka7rq~vP&(K8&!@=4=!Mgr_nFM?nSG>lvD^as<df?c&~=w'
    '{meyM3#j9~0!m4PSQ<er93E&-eD6dPVQ`(o=02^i0W08?G>Byy#Iiz3L$EH6ZM5CIXwSVz2N4CFk_NF%gJ3InFkST$>&+QW0b@(2'
    '*$Xg95+SygR3gp@A}-5GBRr4Enb$hK?G<oJGQ@TnLJ8Of_qA(KPcNP^+S5|GeZwullyrz~nGCBs_THFFpVQ+EE2uzQd#&J<l!&cS'
    '1nB#8Yohm?lih<k=G4e;7v%~}Nr~7NM3^6r7yPeFo}DOM5cPoSU9SLB5+c~#A!skT)PK#$VH{l4u6h-Ok_N$62{8xoy@vekkRT4D'
    'Qff>En34#=G<B`$R_0bW`!$Whd)jcIVOEK&%?}GOB@Kd2gAhK^y!4*PI25Ct#G>~u|0=+gBnb1U)W>zHXsp}ac5uw?M4-+$6>vI9'
    'gAmgoHeeXBT}u7=Jnj(d%)T=fU`i5%SyWDX8f#_UIx_w`z>XVJ0j4BDghM57_weBNmW*k$+1)8@&efbMI3)?fEGjKCi(#YkzMSw$'
    '-X7Y_eL59zN*aV&RL(;@Zes>@IX!bvtl*Wq0IYygk|5lmQp31=Tc=*GU=3eSs6>1EcLk>;M3_gVM%g^*Dn#h}4tIaSz1#1#0#gzq'
    '4w=8yw7w-PPJUZXxQO?Nj*N|~ITdh9N`#qI0`u|3Lf0<o<6Y7l_URy^f>V+r%%u{aw$5`&H%Sb00-4m^x1Xn^tLEP+O<yVzliyFL'
    'BZ!{i)g{+$ofVj7#$-x_-f$fgUTPu4YnTV{-n(PH0#1-QnK{JXLyXgU56aD+*VmKodtJ0r!3i@biz0Nr#EkY!|1E!+?I|?72CRS+'
    'Wlru6aoE?>1(TqgPz^$Tw_WgGz-c8;=2HnGvToUSOZ0*~MG-@~x~PKF&YZw)lo#B`N*{gb=!M#Iw|P+oC(fMA?ppsHD-?&z_!-2>'
    'p_LO*`}0<Sk_^fMiK9o9sjzq{j5u+DI&<yL<W_JxnUl#8cqpX1+Z4O#Y8Qgu_C8N(W;MbDp=sV6?iP9r%cIlD27LBt=@L*`k1(%F'
    '*bCt*mJe8u-?heuyIVXfIAuM;%_<3oH!{t4;Rq?pV1?Oxd7IOxIb}TpJd9xSKHGI^&p`dje#g>JIeOzO0F?y^x2uE`8TH%FhrAus'
    ';M0R4D>!9A!u=}O9yZ_Pj7tJa;1L0*NE_@`fXa%5Iab=?L*Z!s3d%+|`0i+Cd|ho+z$q&d=2)qLcR1|f#rGPbEzJtdsa*&vKxIY3'
    'EGzYeA+C4#HeF6&bzwdya!Z?fRA9=2ggI6s>z=mU6ykC~J*^tUOvL(t`aCGenuM8FB7$e$WzU;I`S4UBFL3{l2K8xBS(7l|N<cYY'
    '!M87W-5K83<KXG>#TA^gCSlH%8yCm0nfKg=ZF?Bbl^dz=x&@%JC}HN6_{0eVF0Jm*2xD^*>sX=$R2C)7z7j;xWplX1<_Fx#ZCbXl'
    'PpAS;S(I@5%Cn-Ma9=p>$7WPxb83~Z0#p_y%)rw3G7HWDFD4()WV=17htgDV%A&+_^Qo}?mvdQ9Oc+*fqXUTwOj(jJ2TLo=;}~On'
    'DGxhQZOOBTO_y-WnuIx6ZW(B!?cn{UD+~E?%`sQf=Jat+m?eqrt)FH?{*ClIJd+|1;A&76pt2;fVZzdY&z%Wg7nFv1?D=<*&5RvK'
    '6mZItgqc__y+ti=ubG#o!s(u{V%My0{;U9%B?)t}M01ylCa0VGUTl1(_ECot1)#DdVLq00W~6m5p*im_*I{eDZcd-)lqCr_vJ{AL'
    ';-(y}LEaFcK}R=2RdC9RgxOh=-gDy``>w?zANPfwtmjxaR9Aq?k_3~d3%B26<$Xn&VU^cXh{)_x^8!v;lVC5CVDuA;-w!Br@13!r'
    'OgC>(p9hsi3G=k%MPAP`@Ma~kV-A4Ee(l0n0gAFJVWyUhJKCIXEo(6d;NV))#@H2{vM3>5aS7s@#3huV`DO0Yk{%sd!6}OpW@|}@'
    'fbO9=uKl<e%yAl10j4ZTn5!iUl6Qcv%f6?9c?@Eeo;_W{DJv4<hE#IgD1zSO2qKCFHP(cl&s6~`OA>BuDZ-Zm2~vm<HuK$GQ-Ua~'
    '5$0)0d&ZFPOEUm!K<=4B+$J0~sLzASYQ)jEMV7PhgXh(rK+mKT)njukY~!#BP+5*JTT7Z0MX@*4=&_N&CDQAu08<tt%+-=mz}M5o'
    'YM#t?IowBC`i}c~PU~weV!ExR#&K>nfAis^1zs5@=B(I*P%1#p49X%2Uenua*876m)>Y%aEJU;CKNWC-%!%ea>ujHz`<{5$8<W2E'
    'NNezsE@deIg&EZL#JhaMF;Ne(2qYN!bhlasC(4|}D_I1>Oa2wq(&glX@Ad-d{?7_fD}kCGN<2-t{I2h<5gs}Md(9r;PyuRZP!>vv'
    '9#G%Nyc0gPVcc;<0VK|lEU5w0J@$@VRuep5vT(Y|c7ysfD9NC9IGKB3ele0zz+s@XXsm83tN?W~sHn8uo?N2aiaI`3d}!p?0^jE`'
    '?W{<c+5FnPeQu4smY9zN(?g9(cH3%D1)#Db0ibUSCtGukTqJWj^fa^yBerx@04hro=51-DylF$;ue#fgCJ!YR^lX3%PFayKZ%cw0'
    'Z#OSJ8+!2yPG@vsSPiHwNq`I+T@y#tt0bXE6tQ2jG=9g`O&b-UvLs<Yf;5_N?}7DR%CesHyWWP`v%d}M)1a~>;pUddYRBFA|NV%f'
    'ry0P4`Bo(gIAv7=-7t~E$LLZeL7QSQwGEf-^;EzqixTc_DZ@)u<T4*%6P_6+JG*`7OF(5&!rU$OdxksCcsJ15gNI+D*0L^-2K8}J'
    'oMj0zS>mjyOnw#3O_q4JKA?7_D&dq>3DUKKmaX-w*YnDWm&C-4v3ASm6`ZmtVdj?h=Do`N67bD1k~qeyy8$XVWl@5*zAYjMYvs#&'
    'k`<Ozx<|IdTLq`ANocgU*U}5HA5a(Bd_0=O0a{&YUI8j=66S5W5X8IH=DZa0LAbXFv}#_$DN7Q|5O`AeGD%zk!W=PZ$Yz0dU|zr}'
    'OA^c8);L7BiSKjBm|^ioq5(y(rvgq{lCT*=Y>pq6E_|KbH7#ElXP9+x^Z=U*P+5{Nb4%R}{qVVY`flFK52&SYw-?msIZ0L|wk{D`'
    'xMQj@U3NYeA2noLcb`xNpt2;ftx&=WlWpfa7f|4vqs=VTx6@*50jR7=xVxoLc!#}5<~(*RD4zz}m_CmwOA=cni9OddZsR@V(~z7)'
    'i?uHK){Y85Wlds>Cv4FQkN8r`C%hvzm#xV9gem})H3`;GVuP>;-j@Y+9!|^7v(}CZIAu-3%q{mkQP1*TAZCqf(<8n!%+sndeHv4i'
    'B-qR(SWhSm_z-TnLSIj^2g&ZMZULyQNw7r{UxsqM4(^)z**1RZ)ji!5Pys4y66S4bYHT!tUR<*EslA2_E$(PeALn$kCLtC`Xc*y5'
    ')p8N}rW~ISz)aM$B`ZK>O~R(RuSsF7%=`U_(l3hM4cRbzMHPU`nuM8KGGa7&UVIAH=BQ`c`u00CFW{6#39&i}o*Z`Ompl|&?|_w3'
    '*pT=JRRAiB5@LlC_DhtJzwdlVp{>sYeTOQ*lqHE{^_Gi<M-p@0x(nws_PwuAC7iM(aolo>wpU6Jzejxg8Ec68oISd$0#p_y%;9or'
    'j4&(}b=md?gm$#dv7ifM3piy>;%wJq5y#EW+?orYBIm<;iRuMa04i$|M`H*Xv8fHdREQ1sJJi%?<?aIgJSdIYz5)fm8y-q*Gc$j0'
    'f46kuj3&?nCg3)w&vTlYlW7tx0JrRhWucb)v&=)(7pMwMkTKDmGI;v|b+L@KDWN0mw527!3Qm|gS;3{=irl3DF2@sb3^hcX9o3vZ'
    '&xtZ86D0PCVNY<8Vh@Oa5%caD)fJpp;xxTTBC1&~4FUdmVSy>KTRAG=v@<90eJ?IYbS|k43`_Ghpv(9wIC17gR>p@j8f<R+URcWN'
    '@1*OOOF&5m<%$HbUykUu^HI#G29muyGA{siGANBCSmQmLjUtz7761$fW(2~%=zgBl%z^}XmBc)9chNZ-Mm!_mdGrmcf>RbG+~HE<'
    '7CT?<!}JQJfkX_k5$+by3Qk#(&<|`K8Buiv?>Q)|qHr?_y+A>i@l}Azk_7CyLV4TZduXmF6nXf5aBYEJQ3ar~CgBd3K){wPq>C1-'
    '%Z@|i^O3I2>C>FDCV|5uK3pEo?F*+BN!>~2eAFRd0jMlW=$5BXw8@KJ-$QdSmx6E@6O!Ke3OHp^0$r4t=0<wXyr||V;wuVui|p}T'
    '6`-;xffh<^3jeU1AD?211`j+wkF7y{8dMe~&_oFY@$UCY7xgH9(NF@;YW3*zpdhOf=5V=*#0$jM1(ec^)@!#wZ&05Hm1PODxZI|)'
    '-Un|)rG$44*o=;j&I>?gU4qD5+}3pW55A?~pPI{O_IPvpG^Z>}kXuucn7Qw}C7&&*wZovr#!<F@Q~)Z=5@vE~0@9*uYS(&2!{kzb'
    '-rx@U3P5FDVp%MqH(2&Q(05MyeZLIXrB1;YV9Kh5d0d_`R*V3k%X&JV*%jE>AzuloEK8Wl<rXtExBBGw18UK<q~hcQ+^*&ooU$qr'
    'CYKXqgx9`Ae1mp$it_05sen@!C6<X22cleMU%W^$#trG?&#t*F;DlL|FptY^ZbqJmf{Skc1biC#b*V%NsH{nBH!NSvD_`e(jhK~&'
    '%^1yL6Z9I?=RsvnVq2|*1>Wmwo@kLdg(u&Uc^Xs!s4PmD%O&GkF}3w_hvLUI_AF0kHK$K=%9@0kTv{z3_!0GfhXUjf`2YlF$L0l~'
    'vL<0BmyDCQCoe9bXoF{?h*mckmVnBt1Y4~HZ3@%!OR>ZTuOyp$tkcC36`-;zVQtcSeWl~xAMd(3nzyWHH-uK3h*f~fvV{3u^0s#A'
    '>veaspJL`zFVdD06`Zmx!DdUfR?HltbvdG*8D7iGj~dkHK~dHvWJ~}UU}a)np#0JUfBY8%B`#kyr%!XrvV=w2bI?Ig7`}xPU{kzS'
    'R3Pa&);+rgoU$q*rb?U{L%c37VlX8xyTCNtmahU-Rwc~lQV32=@w%N*)1yLRlDRvzTL3D{66SDeYa}kax7a+~sRo%a(fUyVs4PpE'
    '(WMM>Sz@kzx$RQS#k1zM-uVhRWmUqAE(PVd;&3^ka7knp^^j_x%L-0elrW!5=E2|a_nioPxt-1)cAhn;&x6XM#PLvKbBt2EbnPzk'
    'MK=aon{R{qJgBTnxY6a>0N2Ul8uH<A2YO_gt2uq1)B0MJK=--i9a*M%nPy|Q=B^&eJbYR=F90<&C@Yqb=lEIgRV@~mJ1CxJZ_uDV'
    '4+=6UeNSUW3E0J&&k|7JFo#WUZBC!(gqf3t61Mk|D-M@X;z@AKL)bB21t`j(ER<lL9!i||LT03xSv?tM4_qq&wGt>ZyEHo;hk);8'
    'Vqc0{1}^>o4*Ci}?F`Cf3E}0}U09c(?@1W~3S;Yh1)w;C@@7y5@xsgZpil8QDbZcBM;KOclFZ3s2|b^dw{W~;LOEXSY!BF!K2Pan'
    'N)||1TZS9M(rw%GIx-kEq3^l{pmr7|zyk?6UT;1xS6!57;LwPoTdONLWlh4oE}8Z6WSrioZ%jHdx%CL{E{?6>lqCuMz^p%*C5&NT'
    'ISJ_Fxx4woIU7^~sH{nVYZ3=z9QAu(P8%T-@O$s>3#b5<H3{(a2BK%z@m)5jwRVU{wxaq5RRAh$5@vRJ2KmIbDVG(cM8l4G?#{qh'
    'aLSSdf_cJ9RGVLRJ`jqT0=fXZI<^E<mL%+L$=+vu!{40t(_uIdQyWxOfXbSLnOz=Q+sk9eJE*lEgY!8w?&w5R0jMlXnAzna2~GUc'
    '3`#W5j>^mq`3f*`7A4H;QZ{UFN|X2OhK7U?mncT;;@ApMS(PxWOWAAgZ%lSjJ|Q0oEm$od6@bdJgjro$<)xQOysPGmaC`2T_02Y?'
    '3Qk#-FssW0v?Wr37n#qZJ&!i@7S-}m0jR7>nAN3NX|m7qdIz;BdaI}IlK$=*)TcpZRl=+;56n1z_;N%c#H>RT*}J5r0#ueI-0V_!'
    'v!{#Z0_wz<>>h|*1S-oCW_BrfhOd@OkKx80!%}RzKyxbKlvN2cyF4}nz=8#S=d=LKWp>OZG@)aO0#I3%ST>|<YB1ru+EqRZ;ao_c'
    'vvrBjgOaREnBC>UXk9zEBZ?Qq@K8O?T9zo_lw}EXyA&1-J@#GZ+vk}DKLNc@u|a(rRF);o?eZ+9@eM;AVm+I@)Y)E82j>N#vMyn6'
    'muF$jrH1#imL>4m6#&cW>uv$4tV?Vk_{qzUpLIE+#(CVel~W5y6`-;%VRo0te=#ZV9n@Yp{1(LvyZ)#IRMsWf$|ZmRz9F6OIHD}-'
    'JJ`A10!~?$Ft^JCi+q|S+>(akm6!U9F04MH3P5FD!rU$oF7Sypm-%@5b<c`oD=TYIp9ht73D#Hw^|~8{VwY}E0S^r2fMV@f;`5+R'
    ')+NNk2%DtlD@rc0#A7CAWiZ@lQ~{_gOi1K7o&l}zD~fTuFfn=RrK>>|fXc#zn6reCO%GR2ayg@-qEFTl>B(yqpt3L_TS1Wq+&;eg'
    'l8oY~^zv*tUCdkoDhm_hNsbjmYc8diHdnxt92%UrwL}T1EKKOR!<M-4t#9AN^D|>OsABC}u?kRGm^da(5a74O^UYB!CcE9}i$G;%'
    '!U`|#?ejN+zIoU=p@oT!>GPPfE@6h3NAFM==6mn0yNjM_yLhM#FDpQ0UBa%q`tR^Kcaw9agi(OB!+WPeeHN5%a9@EUbG+0OYMx=C'
    '1Ye3|Pc*p=B8xj3)aOCX49au~KGUn2FMS-ccg#yM_cv*!uK*NeP-ZW&pAkft;9S2T&M2HDyDM7?Kw$=Dx`Ygz>+Yn;w9y45U=1ev'
    '7F7U>GAJ*Y&~IxHF12F(SY8fu$81gooL1sAEtX)35k%)bmM~uf3`enrtA)M-P&<PHTOzuh`M}+a0n+fU(X+RnQGMMl;KZ3zm}g7$'
    'xxdGJyrQJf?3q_v$X0-o3~HOD7%+P^GhB3u5wnvS2)aF>0@TT%OqUq?OkN9d6J%osku0$q^#=8EP&4ZiV6ntD`Oa0mODw@V`UB3I'
    '4XOfE)+NAF95y=O8;Y!X#KkmnoB7(DKF=x367G7reLWp7etnB2;AxCHj0_$06@bdJ1XwJg`&x9TT~-v10HvYd>`5FIpt39hE=%ag'
    '`9=YfSTm@@w7=-+yZ}_zC2)8mDuzW}lyjy??yhxu`dSI6EK8W{<=itSrrc5-C=qSw*15XhrUF#fCD2PR`SGE;fI5IM9s`5!vaSG?'
    'bqTZ-$L8%Rj*D%<VTwYr#cTJ@SHLOD5@@o--gLf^g_phE+CbJ=*p-(BpdiZ<WTAvihy?q)bbg4#?{~BG&Q}5|%MxU=goqWMUvp9B'
    'A?y0l79#AS@CBf<E<vVCfNU2gcwe?YD~<I`{eWIj1)Q=hK_*Ke#e%vtu${?njVU5?br(kks4Pp6r#R+XbnS+R0l0nk*U|rJov(mX'
    'Rwb6#sjR4KS(gQM!u8VJwnt|;DnMmfV!13~UR9Peyaeax+4+uMdV4--1*j}bER!VwBl4c_8{Y`|=nz;(M|1i-r>sgW4JBZ5jEj2b'
    'q|X)oIBRU|K%xLs7A4H{671D`?d#=$g6NeWIF9sk`aCDhn#A@%0&%%B_AQW@VfO2GE#1OV04i$|+cXJAa(|C^vljg-js-<}Egl_6'
    '6oAT_#5PT0$?|<o-Z@zxymP|gAE7yYnp4&!wg(dPi`IIJ_{dK&7Ja*Y*DU~*HHq!{jrnEz=PRf=9+3})(87@h^=VL9lrUq`xrKaR'
    'd#@4${cPt%Ai>>hQvxcB5@vY`HpkTupm&weD#;)^5f@XRQ3ar~D8Uv>%rjj*x&{-^*0*p;7sytC%BlogFu~IusN}tz4{#6K&vkc>'
    '22}tm>k{sHIc?&&+wWb<#6pifL(IywL46z)Wnn@rm|((CTQ8X?;V2UW5Zn!_08|zx#3YBg;Cy4Q*RjpdVPX#ssKAtE33I&E)wG8v'
    'qTVTC6p0<$=H=I%KF=x35@vV_>xsKB=0|78u-C(g*hAqfKxJ9N3@_&q?}NYHp~hpQDEf9<jxE5HMG3RJ1R!#g^LJlT7$+>5%{!Yq'
    ';Hv<YMG3RJv?h4I%dr!mQBg-1WMeA8lqHGdt)R3#j-U&t$3E6(8BXq-ZULw)NgN9#MzrjnceernpB(Q=MVlYcpgs>OOA^P&aLDv_'
    'T)v!8dVhkLw~O7sQ2}awElI53h4soo*N*so=hJO*#MCe5j?61K&CCg0I-0)sz1Oq=XuK!U3aW=xFQ@`gkU{BtLZJPPG!(Fq-*u6A'
    'VTTe0oG^2;P(mj8g2Kzrr!W1<z{_b)1)L~zvZBkm#cN~tz7)qke#~Ys(V|2JsFgsO*`@xN?GA~A?+a=_1FZGS+x(+~)6SeslQ@e!'
    'S%{Zv^C>{#rUR?(bSgk`21S0^jRezmK|M}l115JdUj-=1ptjpr#$^}wEjIVB86to^%CG{|$)GHj(4`bNS-I4R#Z()s5tD8VC;+vy'
    'Dq&`qux3<~ugh(B3rBA?j~;7N0xHWAW_GEO_m1&Hmtf*7@JX-wl-$nQ6`-;#VQ!cDK93t8y}L@l1`LOgMQ%*doIcGd%MxaGi6f?d'
    '-cp8uk>QxM*k`vjs855+x`dfsT6rFQql1I+nRY0LcH4hcfXcdrnO)L6m$-aj#N~6L$ZXBL08y4C%;ysMOuxlmD$S?W@A*a7da!%b'
    'D*%-x342d=LV54t`=AuSvxi@xXj1Qa6`ZmnfzF7-GW&vwreiWgW}&`56>!RegxOr;p0S?rJzI$L8I8w0n7e}c^Po5j5@vIW$l|u;'
    'eCdVKcrUcrqpzn!i2_hrkT9D|*chHi??L$hw=5OXFYSSA6_~OdVJ4S)JPG)Gs9WnsOvW4f{IFVyEdZ7E2y?mAH8tQ~a5=|W6wV(k'
    't?yNE%5sF6Tpo-zr;FDcE}RgfH;|+cs0vV7jxd)?+?!nDo0nR1y+nEVZl%oKyHNouixTE?iF0o+{@RyIlm&h6Q?SqIDJd18vM6CL'
    'm#E=^SS$gTrU05K&zKZBeTganl~swg_200TC_jMSZo6QE3m^YPGPjln^?6WPmN27B8I$2|BQ`aDk6Dx32JUv7&F4W$mL<&TQU=9p'
    'xi#z(MR<NDpZ3zJ0F`A4bGqDmIFSz+-xuNhv1mDj*p){Wpt3NrX~?i4H9o`ey-Y%4R)F4%smE4VV9K(D8C{AMUHb5iTL*mft?5Cf'
    '5n_WX0F`wKGrBw*J+(Xc<+@88CiHC5Y^Qb$KxJLRj4tJnbwnLG`{Gq_+;&30W(agBQ2;9I5@vKMb~XBjEuUEVtgE(@7q@^^04nPe'
    'W^}ohpj~D!1!5@Sb7|M!*`1&joU$xoK9@@`NX`jw)e<q;S_61loiwZfm1PO@xzxZ9!@l>uAqq{8@K}Po1-^pQ$*P3eT<VDvF7%hN'
    '4roz%REdGK`9}q)tV)>8<r1stbA9|3l$DrAisKyU3`YT{EJ~Qqr7oxwqr)#J)RssYYH)wL6SM+UmL<ePi472o<-Puhp9vlMo6(lp'
    '6`-;#VMdormy`{(neQ7Pou_$&NyGSFPz9W_Dq&8SHu(g-TH-yoD-aiAeKR(<x<P#&R8}P{t;<%@=M$pd<s*H<D56Dcg6~UI0jR7>'
    'nAhd<VL~T@xi?Tte^4aq^&xxfD*%;c3G=$#dhEC$d)fL(PUuH-wx>h|rYuUB)1`*pGseKb`-tfs3mg+0vc5!p9F%_0uRv{gy7Upd'
    'cc)8~^&C)EgV_tJfYZ#Jyi|h6b^-4_HrGfmR6c#k-4R~_C&--iJ?*h+j=OBmBbs*gAVb_;(oz5lGbjrrHpWb9yaaqt!TAKG+8bX1'
    'C(4{mlh`)+&LZ>o6Us)Zc=S#Sd2}RE0BR*r=5%@VnDP~M-dEI_J7hl6Kz?<3Yyqg9L76DA$8<C1cTT+5j*=F!U9dTQo)c$I7E0*x'
    ';~u~7l@j}zcysW~2^~okaFWc)A_+lWCP}P)1?4EM&u#a{R{-i{P|?6)!)(q?jI|Ozw{o%luQ7cb)69~D*<5ao4!*VT9dwt=vj%XN'
    '#ui}8iiDY5YS10Vh_|?$#iv7|?tWcsUco6V66SF!dxKZTzVCWx&*S+-ebgQ-UcxC066PSrdUd6{AhC5dMc!9vw_;wwDGL&2ak-bi'
    'eIJW=QDXIEa>;z2Aw3+b0#p_x&{G=+J?P$YSy9V#P~*U%zCRUk%6f$PTnatzzV8Q=1{)G4Vl+;tHYzw}J;L78HoJ&#3|nJnpA9uP'
    'c&&PK`ZTAkN0`l}&DX_mq&67$a!wkWx6Ne*rz}U9%_Z}UNj8@Y)QLhr0a|PIzV~@fko5@iI0C^_?-x)O@KH>%S+g@x6`-;rVJ?@f'
    '=abcjy_>yh6!G}85bF)9Q@RD5vLazNm-@0MdsFqRDWA<y@i8`L3)dH@3Qk#(Fqca{`lJ1(dr1^Nc@5mN<Gl(@S&%T3%Z=90e$~s*'
    '*4vYpH&0!Coqj6dl=TSnxI7Dewys=iVl{A&Imy=h-WS~pP+5>Lk4ud$hSz;vx@$P5cUhCiY%kIZPFavJi%W(Ry=Vt+H>ktm`9a)$'
    'g%zN(BC#xx*c_fy3$KxR%)=Y@h8oo8L1jt8EiO0G@|^BPIwCqTv=uG%Xiy&qg;|p@i%UIpPPqHNFQ_H#S6%bBlrEAe;FKka?X{K*'
    'yaDvGpv*`jGzq~oT9YUMl{JZNvBahoCFGaO?uz1Nn>jSv<FW!&7A4%{@(c=l--{Z{k4XVYZ+sP)vM8~=)^e}NLES70VyL*BqaU+3'
    'z5-BLm2ij43E*NZ(*@LwY~j#bRNtWrIAu|SAv&QVN_gdjy6Mdoz92ZdZKDEIRwc~f(p;&+!|f9C2~Xeo_%L<fMg^#>N|?hX3+%9x'
    'c;|%6Q@>|!ea?>f3OHp^LgNUHBv9O<;9WgJ0^%9d)>aqB7I31hNtnT<-hb!3r=h0!@g(*cyPTy0Q<fw|1Btb1#x?dbBVdKXr)#gB'
    '=PNK}LBc9xajl5u_`OI%9*egD!{2Ok`ZTAkN5~#BkIgQ;E++&V2dcj_3*au2D8Q8U2(z}-oex~!L%BrdPqn2kD4rcf6oATlgnL^~'
    '^2Ek%Zs8eU{4WRXZquy*l?91of`smSa-D&U=5FL|8+ymZUQY#>vK(Q~maK2ybMKd`qeFC}=tvJL?zp@FRF)&m*^>E0VgoKk5{&zS'
    'Y5Qb@`aGztM;y~5cz_q_z4t*Oz?`u;P7NhGH2*xP^|c_u?ro`0w9d5;O1~*SBcTX;ux<&bnK{AWV?7z?e%ICI^n`r+1G<2@fD>d+'
    '6f@Sm(0tIPapN)d-5a_@qJk4<P9{m{p+h$z7Lk|u=oADw-1(;hOq4Ns|0m3KowkAcWuAW$Fz=6!_X;?z#L0~<Ck!u&rT3feps0`-'
    'bTpjkIHCa5&Y(<@0A}yoc-LZaj>9pFXti=w!ih5{izDVBFAl>?9C2WVgN{9YFQ@`gl0j{=^xf=|#AV|XM$c{75S!B%-2zT0bFy*b'
    'K<riK>phg1FpoQ1nnitAnHPZCS&=YnOVOY&40|tGUD_DZ5fS$lssdD&B)}z!8P@->-!(o9`dqe<y}Hi40#w!{^aG2PQMv=cd-<~c'
    '<7a>kp4nA=6`-;x0Uk@Zd}kh`H}QykMeU`iI_N6^m1PO@w-n*W=IwGu9Rc);mOY@y>Q;ctx&*o|!Mv}=zBt$N1*J>}_O#XtPFa@F'
    'AVF*jp>ujSxU|L%h>A;w(RyqFrz}g@dpg9^{aF_Eb@zWAP+68RgUhqlr0dK=ePTS`xS=<w&7ChS;FM(v6w5;iM#uD}M2vQLSIL;v'
    'oIcNqvnpW*mvWYP3G=?_=CY%pMc+=3EUe&^MG3M%f;dEp_Znu@C-agQGZ4eRK^1Vyngo$G&4?#<hrG+Ni@*bkVH0e2N9F~fvL-=W'
    'LFrF+b1=CTw5*L`4APCd6_~OjVFs77Tx_`Rdq_{~`&ck`x4svE%8G<LT(Sn;7D&8n=6l7khoN8Bv&1VfWkF(Dodj)&)<U>ch>fTa'
    'Gp{%7^;7^VD-vdKdDe+&PS!0jhi68$?t7iwC;*ie2{X7<OyY_J-av(PWXe%Hnpbejio`NSLJ82-I}8C1Vr-6Uo-VokJSWMD#MTuu'
    '^$O)D)TMKSgnZ)@pOxI4KF=vD5_;?i?v3Il%&(w4NADruo@@5TR{<(Z659~)M9)E8B8kP~Z36>H!}Q+y3P5E^!t5=Lka*eaJ1D?s'
    'T<g;3>?BkLs4PmD!KH3z=po<x7*GV8_3k1c<7%ty3Q$>;V6!li9orhc2Ytkk^M?$ZwAP?L4=T$N=5V>KbMWp=g>v?aS_)lLyYqzw'
    'pt3B%KEI36tG{Sp^w_bVRVPfX9~FSgvIIwd<6ty9y`NEwrhBy5jklc&DnMmjf?byw?Cw<W$~mr(0>O8|zD9kX)5*GoSS*2&;JD-q'
    '0l!Sdo6wV?N;qX*!aOc*l17x9zixb_8{?A&v2db8i2_hrmoS%0J)pMRMO-@9wyYS(Mw3R{@>PJ!x`f$WT6j)$oBfWu_4&Xd)6kha'
    'k|^MmWeM}SJmV~{J$et#X+_7>n7nqcrwUG4l{i)zHuRg=Wq0k%#`kpDz~I$_Yz3&SN|?>%95F`i-ArelxxC3{=p**b)(T8nlQ>p$'
    'c}DnpTvCRXcaJ6;eaaS(DnMmT;#eSINe-7<FLP`jV{?HGor166lqCssxtw~3@~C@Wv?Jcb4d`ia?11m%ob(*}ij(}#Cp2h?;OTX|'
    'q&E~&)QHJ!2l^6FGlK%}3fQjkT_STjuEoy5dYkG<qJR@*P98}t+V?CJGH=@(VFF;69u;81jLA$c=fHifHDBn8#gTQm3&biYQKn>G'
    '*~p9U(^_@c!$|w;7w$GGr~tJRs5#u!99`c#bg7bHickY5_dJIRP&<RN)CP->m*k7>2V*D_;iS`XL;)zype&A9Co1_a8y{@H3r~&u'
    'dN~zPl1$0Mh_PZ%yG7+btJNwqt%A8ZeV)_FoHUStJ<l^x{C(To!z(iN<@0Vi;`5wlmLtHLF+jkh?v1wv<wbmlWREUosQ{Jb2!KdH'
    'xjD;iK@ly8S!~RwYgr0FWj!JsE`?*Yz$G%bSL(B)hIHz^0#sHcKtl=SxgxRt9!to6<^=4R58s?V%_(aVV48%UQ+`BU8UnW6ul%Yz'
    ')|`Dr6@bc`1ezvchUIiMxLnVAnzXjR{O0s&PFa#LgG-oJXl|hdUt;s8z?m3EU!p2NWk~`pl&~9=7aj4%m`V;m15&7yx)q?ZDuJd-'
    '?0r0*d(XY^FC7EImfvVjpXZcS33OEgzA^kAX+IrnvZ#k3S8#%?N{~kq_WIAcBsUg^cr=*D>iH-YoU$lk4wnE-%d=4Lfdo@bP&uSK'
    'U*|RoIAu-39WFI2r*DkJTe>^O#-~4TchOM+sH{nt!zH?Scb(WOGXg&SjB`U*#TIbNngm(3<renA?w9_WwMRISD(D`%Q35J!5;jL@'
    'YOo!p$rn)c42H6L%Z|(oIAu-3JTA?kz|$L-Dhd8;zZ$L{x>3O?OA;%pEq$%rhHfx`L|N>@=2XEcD-wF}h<=*0qiOJ6lHjwV!A8Hd'
    '1HJ-IS&=Y@OPDP4?wq)hW7yEr_`DT~0#29}33s?OLBi*r+&F~^=GuBdbq=b4Q&uF*;Sy}d+PaqU39YCzH_z;j%PTl#MZyd&%|Fbo'
    '_V%Tp`a~PlSga9xgDL=(6^U(u1ab5XUaFT@SceTd((RuWn6e<TO^{GfQ2_BCm+$?=<uIu2ElWUUL4w^#LO-=PwZC(sXs332%U(|f'
    'n6e&W=9aMMj=_=Zc0Psm{N~lgVHKRR9>EksZ1_lAgd=){SlaGl(40QaDeDntZK?OCr{eXpoObMb8p6-6WU1hk^$2seBmuv~%KQDv'
    'CJcESl-{-0?Dh0{PL%ZsbG8KYtF&7zT=eo8t-L1Kwn=scr>sY~vnAMAC}V)P8KBX&VWWxKkg5Vy79@nmflcd~Z2Sgl4OwGHe~ktb'
    't(X^p%7TPBTN2~+Od(&2!zK&6`7&asph`exNkUALI3)Ou&|HIe-}z*tg&qy+^PsXM5#E-%@!8gQ(fB5D48x<bx;U(YQ`RKR+S0tr'
    '=e^2&t!K*G-iYlLTfr%7635#%<_Vj<b?be9G>NGf<36DZKxI+Fye$C;yf|#%M82muO!SOukGBd?S(PwzOVo(n8=>%Wi#l>VI5$(c'
    'zU~%)T3^c&$K5SqqqjL-%3=j1%wmOzcXKM>G&3hlZwPvG;jK(!VXP;ZRwve+KF<j<Co7X+++JsXv7NVZ8@`|jh-T-Z3P527Wub&X'
    'blC8vapM^%uEAeVgeu`gnUmF9Dx-bRZwQ84tXIWyb1LAp5-0Pw1fEg*bt$+!BMM{Z0CV<jP@f02Gbs1B)a$E{6u%T6v9Ne%7v_{M'
    '<SPKh85Eg%^o(nrm)^p)kL$5a+zyu&oFsFy7SOT7;qqOL6?rDukoDvA##g}UWKJ4M@Y>$4E3c9S4R|^N*!+(As{46PJ4+Jga0y2u'
    'oV#>@%6=*?Z3H~_6{-MK)+Ef~axG%7E8=}wQTmO2(H-c#ZUv{TNtnSUtQon$%U$=-Bl0bpd|EXx0F^a~Ft{8sse$m)427Qh)elKs'
    '(Ne)FYZ7K~2_VNf$@}nQvp5R-Fwm=1N9F~bvL<2vmN?P8rpA1a%+HG2q@!O*-3wL0DN7ROZ%Gq)uWO-%zSDiYI0+YW2NDIGvLa#j'
    'mUzZf@qbHhY#(c=A94@w1yuklOA_dT1YlU2zpp1<NGJLa=fajG3OHp+!t5>miM}yZSJ#{`D4!?L8eah@&YFbzTcSPEUE@2$XNhaD'
    '!HCp1r~*z|k|6KiAn&1>a4TUx;;AYCBha8e4=QUCW^jqLzyfdK%MnG9+|Z}jD86^T0#I3#aDz*M>Eh9R=cH$tTkQaC>C)H=PFa&M'
    'gG*gdN9a0e&BWL^qx2><`?6aBDvJ_kaEba%;~NWVS&xUu64;zdIAu{{d7Z?F;gJ`^qa#ttcK~thPD2%d%BqApT%rxU)18mcC9!J)'
    'UyK_Xb2O*|P+65Qi%aXv@ObW`@*Q&5iK#0PyF#pjQ&uI+;&KsyFP7K*J87^ncN|c7y2+-1lVnlCEG|i(*R2!r67tQ*s(>f-6q^!I'
    'S(Px4OLDV<yLaNT?a}ia`fhr-ZUv~UN|?zd;@+|Iy>ER}FHun`G5j9ZoIcGdixTE>NkHf_-zAcGOeb`M#MO<}6`ZmtVJ4S)<U}j('
    'd#$<ddJ@LxpubrSssL0LCD=_O_S*M~mTC4?VeUJ2SF}`c%A$n1ToOG_1@95x95AC@*XG3c6{>(!)+Eg3lBOsM-}x>ZAI2b1MuOd-'
    '3P5E|!fY<r+Q-XP-kWSl-#LUmh{;Snz^edM79}`DiRgw}m(Uy+c&{jpMZKUt&*@}Q!h9~tlC}5JBY^0JGXD?ge%%UAS(7lIOD18<'
    '7s|V@Ic?riMmCn)VK%2vbIOv0SR}#v`8{)S3C(#fS7WEX+pf8+0F^Zfx4ER_7M5dQ?z)2E_QprQtVO;8P+62Pn@cjs2#+M*FHtb{'
    'rN=Y@>xgPlp9hsi3A4Gh7F_Rjx^M!LFe;D@*zHgSoU$l!Je1(bUEn2OIK6vg>Q|4B_zEy(P2#vFvB3qrcTYaj9}`zyA!(z65>Q!_'
    'FqcaiyyFV>zVX?j^C~hp6FUo4z$r@-c5gBRdY2^5`wi-G00zAiLzl*Wo|C=oe+LD_O)lvuDPK?*?Z|FUzB_Jd$D<NZGlTL8lX&KJ'
    'NDATJVKk-}-qvDEI6>w_K2^hKLrtPvDB)?{fvo@OBdUTFW>S{l(2J4Ri18&4r5|s^AvN1ervwybP^L=gcIKDeOLUGnATow)L6@7C'
    'kXnh9IbG7;eD4L-SB@hl?2glOTT3|Y%*pJj^a_jLNbTZ|QONoR&KVu{RgmIL%9M!{VV_*033(bg&SL2D*b-2ZL1{Fx!Cshk<X*aV'
    '<w+XSAZSn(piTy5(gZMlLpnF-M88PMXg{3_Ofw4;;KBrx<(cNV98ql34$X7N2JQBZw}MnwCd}(ntW5bjeq$Yoeo<k94*E(sWo5#='
    'E=jMmK4$lljDpGAuTgU_&Q2Oukjl!0n_V7uQ1Y%!z<Rnt=hT>@x4sHcS(z}mOI;!mbEz-+-3j4=uFnsBjj8~Zg$cL2JUG40P<re8'
    'UkFrICd}`0>4qlv=zQ!QqU_PoqibYKKxJXV3@_op^lgo9r7aB4xXCSD+fqU*D--T`2|}(qT&6pqm9*I2$JWi(WwI5dvNU0ymwT)o'
    '7Jcua*72&o<HNn0)8{!s)+Wf^CCtXs_O<uIa~r~eVWr5z9rsm`%G!jPUgFRdV^^v7smNr5CZm;uH?`?y38}13nCm4WkNJ(}sQIN1'
    'X$tOMQWc=GI6-bOalAKoUNVQzq$rK%S7WNcl%)wby|gQ@r`g|cQgr0W#Uq_=vo0Z(wTW=OJjD0PKH3Ozld_<q1@sC~S(`B1%cTKA'
    '%&mD3CSWXx0(I6G(4Y!HWo^Q3FBv>9=!2x*mQ=Kk>ZRDLESgjSsjN+y?<KA|4&CSEE%;)yt+Bjxd|p8+YZJ>ygrdjivfm~2h0lyo'
    'A9T|NF8?2UXQL+9ja%C{>W{=f-~Yt|fJZY8jStH$saaJDlU&_NEL&Z*OcK}tHa3vLs!mMrOS0Tz&5M8nuc1RvskomQeFv$kPNex|'
    'yTb3)?Pt@M-Xllh6Hx2`RmF)gza$$VYujH8+mQ5t<Hm9n^c|q8I1%obq9oZmfx4(4+DMq-@r0asVh5?JPl%D<WwRjPZbt=-iDuF('
    ';ve#a6FWdvc|u!Gbo2}-?j2<<Osl4ZPT0;PwF6X@Cp_Ve$s5DWX>ad7Yux58Yg7MUPW^BPsj5#X`Q0Z?a;fXR&5p&7oLR5uU3`?J'
    'Eu^YGp+eL0JmU$wzx-5i+G#__)7`v<RMjUm8e9;Rb8DU~Za=k|Oqtn<CpLhhDo<F%6P`<@(Hq>KsIJ6wB@x)U`gU-t;)HoPAs&6V'
    '^gX{1CnEAo8HtU$9ua*9sj5zxhZ8aVnMv>c=%GbdbS6{WfzI}*Eu^YEVIJ{WMBd909AMR1nz9(tKAsa>NL6{lV!@$*W3!ZTccj$M'
    'luaXp*bs3v^c|q8JTW|+P{-8fhTTOj)se2zj?O3W+d-=86C?Bq()$i{YdZJ3Hc=QSiQ0jv3KN4O=0ll{%?n{d#++wsEQ}|T*g>ia'
    '6C;EPVw;19x*+AJZrW}`J2BrDQdO83MxO3=O$xe-&T*u48y1Tv;oSg=KMNDxA^4HvSeLx0@miNFi9%^vVLY6^ol~!zLYRn8#mmZJ'
    'n2hz*C59YJq&=XZfbwkDz(a%_zPY!;WF5jhoygKL^MLwxP*_138+jr{OSk6O$2OZVH4M0qDzO6;6;Pu|i<J(TSKqBH(G7D%XGBId'
    '@W>K7NO2_v9-EJjsf^!dydFuUz%)h&Mmk4o2PmnaJc%M@(%kBBn@BOm!OODY{HYG8ZwI9nlmsXlsZA_6K^xNm9h$WhP3!<=6_iI4'
    '$}ktaE=GC%8=b$UBlpvPzJ=6hX#$*r=SF_7@iLQAHvfpZtk>g?Q{>nIN?HXK(L^7dBcir=qcEovV{I=VbJ;DRdIjatgg-oE?K8$;'
    'O{Osj;yC;Bwt#{P%A*OvYZvc*Ns6)0Sza{{9LG6QJ4j(AWwB!tZ2d?v#YVR5m5uE8aN59$3a8u!I@*|q&9~YFOSEZbG|{aco3SmV'
    'xRQdn`<kR-v)n&YVAE=chmLkY?EobeRA>`^?`31(eQyhs7^IUy_ssaLJKRD_D=CRb3Uw}ux(_EvDUzu)2|+t&Y6mH+q(Yt0Sh%}v'
    'afwx5*`nTs{Hr~I-wsk$oqz}^OtK}cFZ4c}oJdrU-^311TEz)~P?3b<mD2kY<sS{^0KN8cEStB0s^SEMI3Z{dc@o@TeMo|HkDfNB'
    '^GNL=Rn-XybwVTfTdft~*yb@6^uf#r)DBQpoPZD~5a$lj`+ek2$#T(kBac7&d8Kxcs`3P8G!Yx6vf6$xPk3@Xb19*-Lv#zMs!m|2'
    '6Ka;tNcZO2Ozi@t&`yB9hg6j(F#6A#r$xcN;e`K|k?GcmC6y=e+X1So6Bz1*N8P-%ukr{&#!QO`pLJ4OI8|)|{e?0FocmJlufEuN'
    'n7i~Ma6F)PfU43&ygkzVR?D{K#m%RbDtj9Z`CNTFNLdvpD58llEnPPwg&WVzd7}ya^!IHcRn-Y1=Y%cyul0LUhW<QF$4r2e^zI>5'
    ')d})&!rz_g;3D^DigDUuMl0tD{PvKl@&tu|9<u<sP48Zw5Tx8FmAlE$yucPxRi5A`{^R$#^A&H;)F8y!1Q;r=p4)E+sj5#fM0+#R'
    'q79QT-w=U@XYz0hqpD0WrP!B49gBfBjXj6cd|TlRv~2-Zl?jeQN4Gi4yNJ#SXM+nusO<;T22fR+i1Xy%7SYnRk2~~v_1W+vb*{c0'
    'psF+xVT4B!v&C}Nh=h<@ANM|vFUSs1qbd_oXp)#JY!6~{8kX-EXxx5>)CN*jnFybqh!3V~pRDp-n=(`Q)*nzCKviKvWJ*Xm2XNon'
    'Y@S0++ddkxnfH*|K&lE862gRHr`4RmHOSjK*{Q^DfmAgKjVcErrFMRQ?J4jyn?OHU$E<V<s47V~0rPJkY+d6(89k$oFgnmv<JbYJ'
    '>Jl0qnAYb8(FG}go@R7TrfGD4NPRo0Dokhy6VheML-z@<vP}D5h|%i<Y6GY$OvJXy=Bd$mFv#2VS{zT%>=bynkgCdrwY>bY#b{CD'
    'U<W06s603y{n!A1KdIiTG+`l4SS-dH>&P!|$TUdjPUB>}J4jV&!a|x5n+`60kxZ#lwzp)<>;bg_RFx*e3sVipgAiQ|=G{jFk00C~'
    'P*s^Qlh`p%b2G9o+K@mnLv%bdQhPvEbz-!rNJTCiGXM(*m(ZGKblBQiqPK-ql_$arGp@6IfOw&wdj>W4Kd>sCD(?<bRi6kmOc~1A'
    'g|x_e4X=9q@KEt^+Q6x*6XAuaNI$lv+;h5i-ZU6W?VP9`q^dkILY_dF+aDIy4*2itct``0L+aZ}eU>LM%rH&4&AV@r^Ttz_RVnj;'
    '{^YzHK=oEZL6+lN>PU;3>u}7>-!uH<^J#I}L8@0$6m=>8O7jW2zx-4^-^vyjvop`O1r$_J9#5FxeRD_ALYbA>FYCYkEV<so2`i^C'
    'uM(OQqIiFxj5;REK4{pv`gV|_LMptr)nmC_CMEX^M29E3)3((Am$Uq43n{LoLY+`(vL22XxMl^=8pr{)1C&%yQR|o^*R9Lq^5g05'
    'mr&0uwF8t^P?Df>SJcW0cwFAv#?fJWKy3hJ6;v8F$5zT^%8S5xJNi53Y1bL**a50)6EKH5G(qt~mt|DFmK2R~GN}!upo$X^;)Lh1'
    '{edtm(x-#Vz!OdE0adk$FvB!zJWW3q4SoWKv%6XKbWZF5RkaC7Z30wtM06oecz8Y+^I}-h9#R`fRdpiFFpYG&mM)eaTAw1j4S=)h'
    'dJCv3PGFWgq)Unu_wjj<%&Sl4cXvR2JE*Encx=!JM54#V?)SZhbPHLO@?Z4y_iZ6n#fk93^d~Bo%C8)ohYZfpus5P{gxMXSsyKoE'
    'Wrf|u9PGHa8S}_crV%3$oz96JpsF~5u|JukEo>P~c&x|M2G~#EDRpcCRmF)g!!(+^9n9_(bQ?{sdc2SvQX5EN6(?wl6WW(+L?xUR'
    'vt!mDGoz!R?*LWB38FYHI_)GDE81*a{<3WxE6hEhsyGo=n4=BN7ih8Y)gevmBvBnFmD)n8iW6alIbujRQmI8z%7*r&$^QNKqs;Cg'
    'Rn-X&b;1LR%#-ftM8*HQWCJu4_R}%D15{NfqIw&Z_?EAfEm~%y&KToV1M!p(cYvzm1e3>f$ObDnOk?q6YVIgPg>&=m;8eAVaKkhY'
    '&--$FyflO9Gp3|=!t))Rsx%QR1sp(2$@O0CQ)T~BvP)gh&aW+;sxlE?n1+p}KC}>J{XC^8>jzNJf!Y9ysxT2=nEs(8ed_xn;``xZ'
    'ARSJ~w*^!cCZbVqpyy9(P%-K@D~GW>;(*!#stOYl!h~OZHZ2P7N%gsqJZ7%XijFOysxFc41bEe&c=1S$3sfvjJi5ddQdO7GHrrbg'
    '0^T2~4uaX%Jfds%oT(k8sxYA;Oc2iH$hvSEGoFS#{wEloGqr<MRVKm^Gqz1)R;1+qnew}_%>_mNs+|yh2dSz}gdt`ur_5Xm?k_)@'
    '?Xt1yHM`gksBZ^V#fiAokYwoWu)Lq@;A@GfcC=HG+5xJn6P`uo^E7wg+&A~3hZ8!D&<CI8H#<mil_$av)1$dLv9V~J=;JcjZXoU4'
    'eLFx^dBP|PO`z-5@#1lLpMDc(=6VySs!oI(=7`qjrG6fIU%xiJik!;f7Eo24n3dc3O4oXyg!_5+D&aV#!!4kyI1z4`ro*zqQf_6W'
    '5ZZZ0hRUZ0a|@`dO^nbclsxHuFdUC_oXu3tjxxK2RMjTJ4ATUrdzCC+scS-X2>X9Lm){OjRhtMq%;6u4*Y)@;FQziv{o6k)rM8f&'
    ';>5Vb30^N2;6XD$H!?UIl(&HTEKX3mVfqurbJzIdNO`WDFb4qOd8Rgyl2%EjhmLlePCB<T67g*B>*mkZS*x;zRIj8`oe;u!r^bCU'
    'r819Q{6{@UY6mE&pd>-fxx%rC{FJ5}nSab&L2yWYJ1MNBa{HI`J{Ke}YVub=A*AhePi!GYg;dXV9oO9Ty^@4oc<ypTI^o0?PFy*8'
    'IN{$Yt-V2{Su{=L0JcA9Cqmx=N-C&m#gwo%kDT|O6!-->9cG13G_eJgR#4fBNpnTz{S$?BNlpI!pApeLpsa%OV1i<>gO@8`Y0T|l'
    'tv9~~Pd$DIs47l`9cDz_vo?4?A&SD&aRUAFJDn3dNNE)(!VYr?2t_n;Z(JIepG-?Q1mzP>>>yRe36J<1>X>n#-IB6#S&elTk`tkC'
    'Ayw502z7#V$hyfTb|<HCDGcZ*Lf-<aiW3kHR%CNQJlW})eIZg+oCrhA0Xp|x^S$7QHutK;ip#SAXA7vRPGG1L8Ur>fdu<Q;Z1y9e'
    'UxNJvemg)_bt3#QJw(rveAoM|m;6VbrW4>_!g-~3kgD<ohCHFo;z-?Fjfn^7igTfdnjTO)Kvj7n3^6Ihp)ASYFDi=e=KL+h{nUB4'
    'fU5EYwJ1u>cB8fs{M28m^zI|!Q<B=i$*MR(kj>50uU_uO36kvAY{AsdvW_jJsyIR9VV{32c+}u0Gm#qFwdslbc95##1c|>tJ^c#C'
    'xL>O>(2Vo<0D+zeeG93oPEe>59-C(=W6|V-aH$hODraP13#qD3M2Drv-2m(9L$YaO75n%IPXTcUsH#qcAtrR3OAYTE{QQII+$>;{'
    'C+~pzc2HHF;ON8zz!<&GeN(r3U$bjnN0YOuXa}e&POzlKtVD?3?@tt6_8{TOcqVYRaH`q_TdcMrvRuS*KiC2ONX-#NPrKl#NIOVX'
    'aY7=PK%9;{tD7&f-ZAST$Gmh0s8Q7k33Y;H8t&Y3Vr04+n=!XY98TZPscI7v+5|Ss*_uU73J1ogY~9nSK_@)lL8@vK;$N1Zm}`YR'
    'EfOgunbAbIV>Y&hRMjTL!-=6hy>z<XFKKg7d8XJ?u`__Pg;dohG{Xsywl)X}bJ=XJu-NT!F25b1sx}dxn9+xsHQYt9!*k;42RtxV'
    'P#sbmNL6t{L!6)=TgQ2Irsn814?;a>Y6q#RPH3nT)<*V?-4}YL)3x-~KR=vW#|}_co(NA&_MA3@x_<8mjF-oD*m3horgng;>O^>A'
    'c7SZ&v&HT&?sHhGgAP7--v&~i-d0l4iRsT%jvwADej*<D=?vqi9ql3Y?WC$e5vG{&sYbj~a$i5>Hg}Wv*a>l7sU4uIK4GCxFzVcb'
    'b04C|&g(o<h#gNW<`z;_p9ovb7;S@PJP|>}Y*|7PJ*^X4Kvj7nR*rk5F#E>jHlFZnu`|q9N*Irjw1ZUDC&CvKS!TJN@3UTH_)+N{'
    'a=Ki$kgEE`h<Jj;(q((GrZPO!X=YrWd7U%0gH+WgMg$Zftn0?HaNMCe(%bBGPwe1S<%yB<1anTc(ZX~G=UTmi+6SFM(H*2d>k}Nd'
    'n4`xdl`oJi?mnF>MMK7I=S*!N)mtT%`UKBzBU>m*{yL?KPH12!M&CiIS5gsA44s=(7A;BK!ucRz0ya9Nc94Qf%D*f?8Tn4};!Mep'
    '$WQ!Kop)*nDXgSIpy*Sd7_y4|W@-#0J(doqZ|6jXQ*Y_EI5O+K@A6YWacLtG>&(aLNZJ94D=3hND1_ERl7|rfrA#y0Ms&2<9i*g^'
    'qATfb>qkoHzsuU}xbAErrIl3nW2$Dk;~rGBnB|f3&(5cDb_*!0pklhC#RALBlNMWgFY5^<O1Hm5>f1?GeF9n(reOM$A}j*GxwgZs'
    '_p|tV2PmlW1f-FbS+*qIo6Y*40&@joROQbjwF6XDC&0rAQ0!Thr$smc*KX_%WT$m(2dFAegg69#`9<rP-uokYE@fATGb){#fi0w}'
    'I1#>>;3PsLvc)6ik)h;~V*mW@1b#b6Re1tKo?x0jvZjkyO6IVRIjd7b+(N4A6Ja`m(nn6&EcU;W|BmU52~Ceav4vFCCy=DEhki-w'
    'wjqg3u5?TIq*8l0RdoVGo$%`n(h&V#LpRav#tb3q<WhS`Re1s}7Jzw-o>ARh<OiY7&UVZwVjcVDEu^YE5zd&vI33X!TScvrP07z6'
    'suQAb0EJbapmeg*xea1rh;D*8RTV-;JEXRds`^A&V<OC&(^!b;Fq7Wi{Og5t_w4{x^$D8#gw8RXdr<r%=@q{x&xzUrs>%~#j0wUr'
    'cf0Z;>79GPqS)9^QGN@lDo;cW13KqI<-X`dw*hUmIaqL9fwzFF>O>f0QlDL6i&QF18K+A~GdL3iTR>HDB8)LT5uSeMwkS)5$?O#T'
    'pr6UMEu^YE5zd%Ethrh5K0uGHMzg;H$@)1`J3v)+BCIjVVjV|xg)OQb{-xw2<yYUaG}uF`suR+rDPGN(w?}HI&hlYX$#MF_>AN{m'
    '6(_<O)BmZEroi`6Ul>?r2p=tE<CHpffU4R=SYyT_Fvy;;`<rhlO#NbXKIU*+Kviu*^2^dW1q!$z^`9N(AapXR9i*x_5sIskX<wST'
    'gf&IK;yxR4_N#0mRmF*DTW0PV({;H&Q^b<}6~jNP@sW{skgDoLSYvX7KAUE7G1N<Qsx6u~52-Drsyq?an1BSxOJ88|5Ppj2O2@)<'
    '3#TehgfXW7Q#F?iEP62q`1P3mnxl{7jJbtWRVTt3Q#2-PmIH|XTV`79A%3IN-n;`;6(_<O(?j%lsr(}>D)NRn<`ysjJ&)7|Qe5Q;'
    '1Mwqb7?$Y${*f9m8pY*j;e~bThg(QheIlGOaZtF>4qcQF!^3pi_zgG``VLN2o(N-1YE$rQxM;;xKTmVAqj5h~sU4)MJQ2p2hzgK*'
    '-|Z`<(S?+EpFdG@%2HcMRe54WJkc5ROwm0k9hUO~T?d@FZwIF;PJ}Ne4Z>`lSa`$5*OKndeF)1*dUuej;>5^m2hH)C`vWC?%6-aq'
    's)t)JRc#_nF%hMuF&y7>;*q-`@>AM5Pdh+WX=3C~QMFm-UG!o42b^_YZhF?WZ6Wnpn~-$H#33y!P7Ad!;`0p8jqn)w?EodMf{H0o'
    '9Jn?F_BRT$ARm4^XF>TMP`!d8@ejNhk#aORF^;Bqe9*Trg33rjj18f0ok5L02i}N=obk{doUn3=22B6qvr~3)>0QG4Yyv!>c7UP+'
    'D$FqbQj-Ufd1p&UdmF0lR5-SP;tDGIAA~?MKU(PKHZs-44%!c>9iXIw3Q0n{<kfe-R7JxJMCRNLjuT7l0HqaFd|3*Zn?!AK_xYit'
    '>4hnBT4lG8vPvpc3D0Hoxb%BaDw<vs{_UO&Y73{TNkmPZm}pM%-dCdfrJJ`nQ71m%K}xGA0XZCcSy$bI0+?5y4I@2WeOo|PQ6j7`'
    'BZo~lxCN=!+n8n(>C|&LeLJTrOn4L+7DqOh6y0XLQG#l-6oq_xFL!XN!bJFBqIp1(E0kgJMA>w+#|Y<79<_s1RVFYC9M?XT`|S@3'
    '<ayNW6ocbbzXeoPCc*|&8bM#@QL(~EvRfjSejUToEu^Y65qBB+8)Zuk$@8cmVjl>f6Sad=RVKm)lRTSEm$&==BZWRkLj%}2Cu$3+'
    's!X7N@ha4_U*dj`p1)92>>D1apZI(Ws47i_3nuxeYI9H2{oN;#ai#Ni_yr$Q-%rY_G?68afr`s<LzBkS(o|yq*Ev!<KviiXTrfEt'
    'j-t>3_eI_)4(IG>L#E?fwS`m_C&C7k0iaQiyx)^T>ggMTI0R4U`3_Q5o}jqb+H{sR<a?|7CYUnPKuEiaL+aZ}Re2(OFbNv;wPUHm'
    'xXj=LmCrQC7E)E82qR2k|H3ls)x~@iVV}!;{TnzD#ST(cp9m*R3b$rjw#p*lrOj&R<A}5cR23+~2$Pt(FSm#);*TL-f3f7eQaeCZ'
    'eIkr71$3_9=nD;<6IAaFxSbGv2dJt~NIF+3MTl-=QSI<A#imug9!?uLjjB!vNJo@0Q>nYK5Bwmp=7u7PZ{SF?J4jV|B8)IurDvrV'
    '@1qG|nKnBl15Yrq15{Nf!U&TUvumugE*fTAp1RnkM`ugN7E)E6keKRFofq5UNFkbLpL#64KCjdcP*t4JCXL#(KVS6s4dlxMm|5^x'
    'jBO!R#fk926r+@8jTXu*&$};d`%cQc1yt20!UmHwrHy&(Mb0bN(k~gt{j|*PAXUYQFv1KSJxjg!rXU2<1K1!vtPZFRpsF|#T`Sr;'
    'rQElO`&e6c!mzN~0ks2E6(_<6Q&6&jWwH21p|_C%g><#ki@AZ6zebf*PKoweK%_;|+r||eGQ#N`+d`_!6XAm?F{Ha*BC23G0}4NX'
    'YA5U6L8|H#;e_cA)qt$@>LT)+LQ?k=#qGrCJ4jW1BAhTIMo2kttZMReLeb-R9H+@|3#lqlgcD}W*yI*bx$m3x$U~<=NVrs-h++q+'
    'Do_j%|8uFQodK*moX3x49+HM+ht#)|stQGTVX}Xz`k)^ZBRkIV4hEz%+O~sKRVc=+cp$I)T!Q25bMe^Yq*Gf!RfQt#Fr&wa=fv*f'
    'NL{Wwz4z1Nyoc0hi9*v3Qvh{de!3zxd&ks6^y4_TgH&&o)HJfftY<8S+af}r8JB>b#R6MM^-3xv3P+I5V+%VMKXs|KTlb@N>|C~x'
    'f=bH2EFts!H+m!%5ru#0sp2wPJ1ZczkitqTt*pkZ=H4q5j92Ifqw|@;-U5mWs4&K46ir3%{zOS{fV1QQd`70WfZ__u;|c5Yk?Ks#'
    'L7XH-64&1$wSkmWQY0->-?6IqLPIg0ev7dk`{)xpKxqXP`UFkqorS3Z_u1yxJUl;f-yTp_L7Bu8WVeT@v-Mo5(nK^HWe%fnXH=Dm'
    'FvS!$hV@DnC7t$2(U-}`xp@aEsLDjH7$0rQvJ2;CmZ>3iVmr+fdq`DfB0O5et`?Rii!*io9&1Kt^Vb$qRhfWLCNzHXWtR&BPGQ1w'
    '8svc50ICWTVT|d)oaE(qAM*)Z&6p}iC!W{>stOa~i|Mb_wB5OXq{IWhx#=4HD>x@=2dFAcgfFI%_1sB*ub%tYnFb*Z0FG<@7EV=`'
    'z$|m<Z11DR)z^7SeGNP5A+>{4)g@3QR#?xShj1HB3_)$P%whgN=kD7<stOa~i)q>za$@cFNGV)4R^tER^i6C5Rn-aqwt7>V3tvGN'
    'qHLd|ZzQ9^IZ_))VO1w6CPiZpTU>tkV{L*%Mv6!=;C94`9iXZ>L9s`4w2=Y&Z6}iSYg_=D5uMiNEugA45w@6`jz_sEbP@RBv@?js'
    'QfK&j3#lqj(2v26Wxk84^~DablSgfVRD}u77EGEurSzVZ>YVB5F$8kvdAE?N%0xI~8dFcLZ_(Vs40CzBpRIn@6>T9^wTWmwmj*gV'
    '<nId|GB6uv+aRKoN9_Prr3uasOs2Mu&Q*mojT)^V534<(sxlFdm_yrKy}MW|<pF$7(QC}p_Y<D)AXTM_u*6jAbNB9I?pkEJSeb|-'
    'pEI?A6jf<LKpG7-NYA%>(|Wj8c9;nsPs}Z#sx;x*YeyTH^K5imlkylM4AVoti1kLN%DaP9)h411!ea%9sID(MTqKru<OMlmzse!C'
    'fmGEd!W2`1Fw<VSC*`-}?C&#1)ssx^AXNnl%}&ht*sEtsbwIl;{QTQF`os=URi6k`OpDkrrvMgxm^gC10L>Ad!|B^ORdpgfF|}(d'
    '=!*^)l|EaMNEA*N<`z&@od`=z8GYtbi<xWt?}}c^bnKA&c2ZTI2v5v`HftGhFQPl2*)Z!LR!%e07E)E72vbZ009=T&)-RJqle7A3'
    '2P3Y+MA%^}7>x(=xuO407CK@F*vWTykgCdrg)$KzDQ!RGey<AY$dZ&DQ;<!dsxT35nEu^MbhqTbl)Iqs#{-4oiFAU#9i*x-5pI}D'
    'QObzW8yA-!rg@dIdB%G#za6BiFcFOq;y-r^6N{M7BfShLlsmw={C0q<!h}cUV?<MPyiyB4IEn4$NeWocWc(IVRhkGhOvSnJdga_V'
    'I`y8F4)SRI#OOOnRc#_9*{IvjzUKR8BpWd2r2PBQ(}BK)RMjTJ4b!Yi4zw+@DfMe_4)c=W$$Phu`m9Y@x?!3~)}$6EN+D-?oohG&'
    'A5P!TNm}I;`b3wx@R}E-r1zOjshukC9#XxMig-e)=ayu-jVGMWHPJS#F&|PpNI@kf;{P|*zH*#^rtIH(U^11nntTf<teh<Mb52h8'
    '@i|{I((qgG6nVFBqQa?X8)j?`SZ-+>exy>E^#_gQTzoq~aRmj@T{+;9U9;HbW8h~>&f>!@q@<GaV8R;UwEf`4<<}^CYSDlDv4q<~'
    'N-L=lC)5uLXXD;|*asfMCryftdWX~oQdUW2<An6sd2OqlmvT)}{E&LQ`?i3p+5}8(Vva!>ECz6{?uj7`=at$3N~<^#c9@|yWQ*m!'
    'IAQ6Ul>M?u<EVW*Kviub+%RKaZFZpFKT@5Ergxv;e8*UO3#qD2gd3(ON-bw=7E>KgsppZR0pl20Z6Q_F2{38qe^64m)%*Of`UlVH'
    'Lr~9uPxHhMQdOS_KTKtvF6VmxO7$=XiI*&#$xej6160)~F!Tw3qwIk{h<Oe*ol9F7Y#&k^NL76Tv)G}axu0^8^)|#Y&(twO-$bhF'
    '6PVqY#<`vMevh&>ZP^NAJ*EZDnc6|B>J#CJIr_MEzUrdXYqNFCT7LWk>f1q8eS#u7?|trky^qnY!PSlFA5)Jx(hgEq1&VOQG&Izu'
    '(V}%B%%L_NC};847Eo27Ad;>`X|dgJXFEJPhdCi1&s00%#12qZp9n|H*dd<^`Mokr{c>&QKF;jn7EV>22tQ1dOGl!GG;4iaE2aFB'
    'JfwDzs_FzcX=WbQrDZ+b`)fC*y!$*nH$8XX4pLQ~;E*Q-d8+8Ex>TRFtK58jNbMk1<%uxGGy~4Kj~9U-%oR8TxVO{ayoFSiC&Cre'
    'lsVfca8a0oIWWLIJVFkr4WOzz!4XYFkTBJ}g^EtIB%<2eQPH=Ms`5m*Vus(?G*Vm$eslAoK{Umj$ZrFvQPl}a6Dx%FKu3yd@7zn?'
    '+3d&@J4jV|LL#2%Of5z9TTmXPUj>mqt$7ZpZwFQ72@y%-kE|drW_rgJPWb;%a8A??P*tA@PfY*vbrw5lA<fQQstbDDcOvv1q^drl'
    '*^N16qE7dL9{|r*bWs24PnX{gQdOT&kRB!ZREfZD9g+Ybb-`GKbIKiiNL76zY%wh!pw#>pS+C@N#EAUH*?GMMRFx;f7SpVmJ&@76'
    'SS|pIvC-4!vISI?Cp3#4I;&2LiabxtU~Y(~BEN-G6(_<RBtX&9VcSz)kQ6Xt`5qm;@B2adw_QPnkri=EL*RQ*2v_R_ntwUxMC|}o'
    'wF!$pq=v$41s&n)?~?&^Y^=6`s@jB!_@AOMjhj~nA)YMfXa~m%*h6XqsVYubZU8gR1GA9(glTHG4f}De-vX*?6S*H$GmG+|Vs-iT'
    'p5b{c$i@>+>>yROi2>3=$V(^1`v4vL)sCc$a_YQWKvijCkX+-3F(ps;BGMq3OZ5imktTM4s?x;pa00L!tf$&&O#9NoJo?$CyaiO1'
    'CPsMkjXvYP+o?A25Pm>jeiX}a&XL+d>a#d8oPr-I59hKKQ|{G1X1;PMQ9k9v9i)1zq|(XCW~qY~;RHx3f=Z(uPCGdD%88<DES&3?'
    '6TJ?KH8pns1I<n|(hgEkNrgH=GA}k<Ov(EfV{>OwKOM7MKw$+Hm5#X7C^U-2iSi&KZ%5R2+Aw!;qQa?1r30_U;|orBjY^rveRkBo'
    '9iX^^irwT5=B0KoO)y$+yjDMT{rEO-0VNew1QUL%;lp5JOshqItGb-KZwD!@q(nNHPMZR}x7fCk^Q7UPsi(Pb3n;6gJeY{4@|h&9'
    'f{850ccOlZyjwU`Wg<4iOUUP0#JCq_aZFK`1dkQ^4o*;Y2}oU{X&N{8MbzivM%H*8*D;0LL#iqh5Wxg`Oh?P84@aNPf8io>94EGb'
    's=@?__>Uhel9wH09zu-Vju<iDsT^(rRfUN##PkQqa$Seqw@onkZ_C+4oJ({Lsc$D$r3s8W2lO_57VqVA7Ml?V#!S^YQ#(jiZ6X{o'
    '{U55S%C1I4$*;W((3MU(eh;ZCPN2sVm|H4W8+r}KT!PcPdMb4a9Xmi(Z33fjV)&Jq3p*B<ANnIU#oK;{uD5`y+5|>C;d$&FXTwE2'
    '(HX9Ig3k_|Eu^YC;op|ueXO~Cw=YhV^tqqcpQv{3z73$T$`k%*fgw$|oqp>zhOzx&?!^|t(}KQ*RFx-)q9}D$+84?!kfEj17&kqE'
    '-xg9;pCFGXP+)E+r$rxToBD)*K0I~aEugACK@m?V%%yrOd19tgjl2IHC-B<=s>&1Lh>1<+L>nyBiMdg)cMsJMrVW^?HW7B1envP|'
    'r2A5bhZ4QbU7{gV98eoTRcV4fmhi9-Wx1)ypD2S|sAtS)o^2DUs!edj5{-Z#c=xfTxpQ27GT=z)J4jV=BJ3~`G>f=>VF}0LVPAUf'
    'Kszb#7Eo22kO(Hijy~50tmHnPLvJ1-svJ@qNKq9h1mZ|VDK~xj7MEXZH0PxF?2+F?s)`dr;$NrVZM+yg1XSo#LD#S?J@Ld2P*s}<'
    'J4{rwEJ}$kj+70UuN2}5(RYBV+Jr<nK|Gsb7d_}aZ$~x7aZc10P*t1I)Fy~CPg)hFuKvCT980ARsSTv6JfYb-;h#Akse3PA^LUSD'
    'x2CjyOq;ims``YcJ|UWU(n9b{k$rYIoAC+JcaW<3gnB$dqxIaYaxYH|!a4d8K?R(?u^phQJP~%7sF(C`-;Jb0`V2XQv7bq)9iXZ_'
    '5q_Bda$$H_b-X7<e7#b!gW$BGZve$rov^7+P|B{0`FI(Zj^v--&fxVHQdOO>=)~kU*9I)^KBsAON)+ih8EgSn)d>?pHJr|TcWq;_'
    'Rlt+5IV)x36nb}Xs^Wx2FkxxKe0P?FHP4dJ=;zYg!l>#JBXkKCl$=$&2NgzS32F!(PCGbNRU&*aBj!$BoNmieqg}Ol4<n!*QDO(E'
    'DoTuOz=R0chxNtP*8wzVr2Hv6QN#{VRg(xGOqM~&OC!wD2rqA0Qt3yL-9f5~64A3lU3*3fa9@g-xg?@>zbVJ*Y2HHWlPZBNjWDSj'
    'rR0G3iflN%<V%JM*iUS}ft0jLD$T3<NOzX82+)7Hj4`L{=>hfapn3%rs)WS8$z`{!r?}ZOVZCG47Z0d!2L%<BWZh8mm0FxALJW01'
    ')(>buHI5yiu!1u2zoq5iO}{P0`{#;X0$B$^5j;m~2PrC~dWI9d=XqK5U>aVXeFJ)@lT7U(#g$YxO<;_x4YNhoYcuQhsKV%k=vzoh'
    'C6#^?mQ{|`>KjwcWqS>YWSq2j2Pv(jJfIL5)5GO9^7F_Lvj;PpG5MHbZUJQ#RMa{A^LARTzKMypLD`0B{_UROn;oF4JOL3;c%~F5'
    '$`&)%*7_Xf#qjVuM`{Bpt@1?pVfv9u^S;HBBEzd9!gJZ^;q>jCsyY#Fm=q?n>yc~o;4tR|II`aZY6qyQPNW-VN)*`)wm4BZM+J!5'
    'I17}wkgD=T*kO`~-q|aQvWO;Zk{a6i9H||ksycx!oV7iTo$JPL$JvJGygi3+5VQT_kotB~Rh<YoOhKkx_^OLiugB+kKgLGA<5$0h'
    'R8=R!4U@RX9%WBx7nh$lm@8qsu$_ja9i*x{5q6lwHl{hqJt(=>h^qK&b>fK~psG3%c9?#x_1J!6ii_c!#$)<dbr`4Iv4d2VC&CVs'
    'TdYN1CQ}Y-?yrokiD!%G4p3Ir35w<kmg!@#nCY-F9qW6WU+n?)?VzeWk#3lDIcK-U-PfRHk&%WXpTKVisj5$e8>V>9i|eiArX+Jw'
    'k-ueUTgMhqRi2;>CxEA)#Nw5bX*JfO6|<kLcL%AePlO$&c3@fVx*phD%ICvRpB+v+I8}9mW7jyVX79wUjKm85y5)glIp=8$r>aea'
    '9VVI1P1pDHp&mg@3l|S01dlMg15}kJI6E<6ZsuLMP3T-s2@bTMpl=JPDounLW?W|3+q_WwM%Sj(#6anss2!lHG7+&kt6zZGymDVX'
    '46o)aNj3C8@yN0pNR28@NJ<k#Ijp-Vb@av!Q)GwEXZ~;tsj5u~h1%yFQg^uT?`uuwnPTzC;<U`}AXT-AaKn`0FHf$vk^3{{zh^GU'
    'Ir{k?PF0!+GfYNLliFHD8iF$^HlwcRmD)k7N)zFRX`M~iD_@6Sk6G&I{$s*1*WN;^iW7zso=6(&t{0IX8W^=K$oua;M`{PCs!kXS'
    'Y9zADdOY?pq0ML_`&|yG4WOzzVJaw2DqkShds1yok3~u2`a^01sVYxIpdejG#uN9RJJ^9PFEaB}c#2XxNL77e^l-3}Y{k4^B#I!{'
    '{EcBd8((*T^4F+>@^HdGvP9=Z?~5bl5jxG?Ukq~Mi5;Y>JTcM~E+9qQ*exjtFUD*e)6RIBk#>Np>V*H$=po&V6>bky1DMVfT_yX@'
    '4ySMDRMm+QvV)Q2+QZwv2{7rL&zMK(be^akoIYz4{iAs-L1fc(;#QUogZeyBp8B3VY7Zx_oII33fu3`P9q(_x(RFTr7z)P+cnc}1'
    'q+&l&_a8liKE5x=M=fMpVm9d~Jl{e}D=Cj9)c>xWg1S9Y!2VlOXBy(vPISHllvPk8E<c(^^!FN4LrnFecRL%3wqP0sQ;%4}Ly7wZ'
    '9bwZ<G+WPOl>R{aen`DlNKxWI7UTP%oKdn474y6&$=$)IS4JF3u3u)^b$+iKHJLlGJ=z%UwDj!&1r?M=y$~=bZti0}kIJ2BIsgx7'
    'htw8QSV?7d!zguDTIIQzZRg5#<~a6%s)7VW5Fu@3yB^(_cEO*g9JOwyrvYgXsVYf;M-m>BQ(rpE1}TQLt&%Q$d?vP#s*(gmJR!nq'
    'oVX|$cAM4Tu#&TrLG1ul6$yxL3D074=%(MlQZ}Yz-|z!<q@yjMsv_Z^6=R9l^i5pdeUiQDJt~fPnk9CSs-gr&JTbJ{a>pxzp<JK_'
    '@&UC06jWgXF-q`sjnphE9DS}Ag{CHFrN9<aRi20qdVb8Z52C}O|448yD<3K79!}rRsj3qgy%MO9edn}zq{x3q>V0$)sV$tUHi4l{'
    '_&b#cYOLxT=@d6qhnXExJ4jV=f_mtE)8Z}n7~YjRjN3GBX&@)#-2tj<6BOD6wK*TKxcOo+W&Rcq&haEtJ3v)wf<l^zKJt|N?m<P$'
    'n}K1>93EYE2dFAdP)HNhI7XvZZ<Ih!6F5J9bTp(bpsF-M(Rm3iH@)_S*cYQw>HZ5g;GC!(psF&#5loD^La*QZfgm=XLu^j!SfB3z'
    'g;km0R3?B2rEl0`hV9CuJnGXE_H7|ml?e`IBI>;DLEHy8!-?qCgrgztAXT*q)>-DA_4xZUh0$d;*IY9n+o~<3sye|Vx@6PtXxRP8'
    'P4Ak$=_5bn;k1EM)g~mu2}9KfDw4+>x#ZeF{HHqc#12wboCvj#%zyH+Tpx;2x9ZajDWus$Y6GdNPDsWRbd?70`}=TooOAO=YA+p6'
    'c*x_q;eUR8B~<h^R8^ml80PR8As?YdjIKJ3nicT1*N6T+psGG0p-(8xjbC=3_7e0gTzSeR`r3K#--D_u6dFbo0BKxh?Y?=!KiWKZ'
    '{tc*{Q?&yXRgFT!YT}0rFf1IA#DHdxUmJ(i7E)EC(1<BiN4Ac^BBmfsW9>Bg>j%{qR8^$V81Fz2D{|=izB=VcDO0Ke>dzV+R69^r'
    'l|n<MfDZGivW4<zbBA*CumbR)+JUMH6_$06Om9~!+WfCmo;*XL4(9^gf~qPN>4z!T1t|9cAPXU--41kCKB#t}s#1kzSixa++4`c{'
    'kA6(!^xoy1sy(QxR1uz-v3)2P>aChyJg7)DH+Ctv6IN_NRmF;Q&sH0=_;G&$Vwh`BsWwWA2h|Q#RjdeCOlAK_={R<;09tsaX9U>r'
    '#k*+#8dO}xiuA>lYyG(4dMGDOUUWRac0<MAgQ}_(X^hDxv*r2ztzyt?^L2Ar=TvP$Rn>~H#+2sSRu-ZbYHgci1VjBB(o?M3f~u+&'
    ';f|@I$h>y60{_@^bRK%Qa{+Ea^;xZeG{_X}kj}vO?X<)VW~|;2`wQQH^6?o{8oj>`DgGKMuB1Y&FdcdS-7A3H2hIuz%<hXr>ibDa'
    'C6z(NTtdEZapqx-W+x|dJEXRd(n>0%3hgssSi}_AuLc+~QL%$+2P&(m(&376E)uv$MWey&fN9oV*^cLLLNyAiFv?W_n{vh8eXZUD'
    'M931bAF1)myAb~(Qqn3Zk0}h4+uD-?`X&DXox5)frd}~cKq2AS7nl}zA9>c5{n4HHMCp4_K}8ik6;XY9aQBh)IrkBM!RJtIA%&IH'
    'h`W!c$5oeAMX}BG5v1_SjY@wLswz^1X{HGG+&OTcPa)^XZ-WtE!2z`cRMjZLGt-0g+;M%sZUv=bW?UYid+7J%+b@y|qaVM%E~@ss'
    'sH#X2rkN3e^Kyd``rkGmDkK?tQ0+lgRf@39G#EM_jT9j2w3lo4bUFS14XY|sKsIVBWb=!@NBsBy233_QFry0KbWyU^_=n`Aju||^'
    '{rUe3E3HCBxMv#q-<4aZ7Om=0sh^t>;n|Iv4^aL0?~GqxiIjYeRMjdlSB)BHtPYC{aQ0~$H@`yRklI12N)?!vSAM-MuD)@VecRAy'
    '+dKb%fmBtBFwZoOfznm#OXmOF?0I2czfhiU0k;4A`u3dQX`HG=5!RX7o6MEui<)o03i#^b#38kXRMjZLJ5yrtHy$Xt&!$u(VvI8t'
    '-i(LT4pLR32=h!aoJ(OB&6)m6$=s*I2K|uQL8>Yg;ht$a(m`(F_~L)=m@G##JD_%es`3P-+Z7uvH?W|>C8ZB1+whRuL8__~VV^1L'
    'rz)PQ`y-`YW(7;p*`71CgOpWmBK$MW^H^Lv$T7`@m&3Y%oX~Fzsj5wcf2OkkpO+i4qjzcEeo=HJKBRV#s@g>OXNvOd<XgN`W7=JL'
    'z8>ZXhtv*IRiFs}Ow(w0Zf#LUbd<9l+M$BHG^kI1ID`T3>wvPafvO5c7-%ZR`mdHN25tii;_S*KZM0rR@cB!YxN3Kk`uxw9-=E3p'
    'e~eTV)W1TiDikr1qy5tS=pL2*xc#~gVR&Iw`T<IwR9wH>E1?=+Lscb;aM1L~8^(Hk%H*;-Kp3g;zAP!L|L|8jRocFI|5c4595gM~'
    '8nxw1B_2ui>0C?ehwAg!#Mzd+8&zzJ@wDRl^|wj2KSrv5kyI5aB*#a=G6?5In68%7AT)bBUzU{k&-+4fuzd|q)g{6-Q?)_P1Kg|U'
    'rdOe(`@alJ_5Aa{!v5=aIQ^si0Ka|V;rL^u=xbrbsJcXWW)6rjVcFpqOBCl!ee-mRUKtg9^fqj?>(~B5sQ7DB*rN&)VVbF<L#~Us'
    'zwyla)innGL%lRA{OCf4eEq5~gi5}a?~bZWglVQq@3_AI!qQ;I@BS5xmqtZ3<9#7g`nA}6RAnMOGe@5u+lyZGu2Y!+5BJuMNY4hv'
    'AJL2GanJSZ3y~WAYow|)5tf<p_T=@q&}Dg=o0{2JcOyNUlzdS4pBAoP;dMw^``X5cQKbn>YbzQ#hP^1s$Fx+Ir3@OZXM4*x)gu7e'
    '%St~!{k61+|4ml*$5{0*1{GDC2+K_Ufoc&{@RjiTH)rdi(PyKwkN*S|`Rfazg0HnVkE%_CW#$0WoQhhsH%Ak6F7);92GM?-Rr#{z'
    'kFpvcpZh^OeGe=B6;}AgV^!6O@XRzq%LMrTSdBJq7zg=PxOR7WKB}sZybvn-;%im4iuBAJCTkxa(<mgSt7c4&zA!0mAD_ZMkXJfX'
    '{G|v`TGcATGIKEJG}Mqqb*f!kb9!uedu3Gp<3Ht&_*GsBm3<9W)hY%HkqNb>!$CYyoZDh^C)Ic|DlYXW`5#tlp=kd2^=Yj7_?eZ^'
    '*H~4t!hdKF!-b~F;=NkIZI<W#Z6p2z;W~W&UHoTZ#Z1H>9C7<+R`M&X^ov(ul`G=jYYuBXsCxK!p5yiY-RO`PM^#!m;FVbE*I0cP'
    'EHGU&O`AyW_*kUD;SV!ygFVN3ZCLoH)ulfN%iwFUxWWp}LV*@J@Q`_^qTz>>U!2o_D<Ex{y%MJ{1{P8|u^*~%oZwY`e#Ds7m?nTA'
    'uZ@cSv<&#?P>nCf6<SewT+t9ZJ!qtr(PtyA_Rg=)s*JF{()IVX7A0v`S%qFPsBtQ`w{I1-%QxNBUpu9jhQ(Vn+h2h-8h;JeD6o14'
    '7Itl0zP|+hBW3}z1MzruR#yJ;FT|>UsS`$etE|v3L8hLp7D}LwsDk7!q*%A}(y-(Mio*0tu)cP%datk|xZs!-jZ?Fz+jq(iize6l'
    '(wBx6JtNJ2{Q5$$;A^RHhYBlNHT&ohJak+X8DZL;#@@>YFU?A~ltX@v)fW#IR#qunkRYwPEd05mjM_O&lwKVbmB!jvLiM#suvgV0'
    'Y&6XX64iY@WPs*?Tys$R;$om~DP8>%tFKiVrB~4+oHPxUGQ+yfhADzyy~ckZ43$2em3@4^0O6I+6@E>)=vBA~D@_xfAnv8x2o~oa'
    'Z6sxt7l*Z@()dfTzSb2jy-F7l(uLn__Mj--WF}!PR*vb5v-&IuzS6m(FNPLXxd<=K!C@5_$<uwQu?>&^vclCRJ8NE;)!0%N{xw)%'
    '3@)m25pJ3$Y}syx3z<p#G^~jK)ak`xm6-jNqKhxKTR@dAFr*6+%|TyYO!2_%4fIp!U&5=y+EK*+HCSJ}2BAt9;i&01+_j_hK5QQ-'
    '^Sx?L_obB!Yk$}c;ZL&q+ARoGxxgGUrMao#{>6fJId%I@IL3>kdeL@^AzzCFq3RYGx<%{=TaN>qOlM8gA?auP(x^WFRedQ|@-<df'
    'xCnF2hy_zchx^!~x4BTPlNb(sX;}T^&yF(wIad0$st8o!BJ4Heu)qW1!pO2cFiKz|FOKR}J7b38*H~5IA`CW1H)u<p-C21Q7_1y<'
    'du^TFSrs4ZmpWHpYdM3eTTtj0VTrZqjJfKJG*a|<X-cC6>n}vfzIgjp$s#JegD&aF7P9U2TY&AgdzcLV=b!#MsJ?jlRly<*HqC>&'
    'RBh#6we6Q7%rN6uKUY6ZzP{4!_cfO}!75mU3K*k#y14ypdXg|PFugdYSM4<$<7?L+R;eNkHa+{D{h7GX0CgTIik$Mb@4x6D{_iHg'
    '_DV_B*IecVt5m@`a~hMYf#m-B8-yvYJJg&;d1+Rkx?wuJ()IVnep{?!1&3H+{(UWzDvbTiHt#@`7f-72bz3;c7oVysR)o){|KQBa'
    '1xVCJ?>3hfV53)ORdl6S3fITik}9lXML2B^lqmw<hwBjEY4({pDtvKN{}pTF*H=RIwI&O!T17Z*4p14(eWj6E4aiAGkL~%@pR3On'
    'S@KG$)fYz&uxb^OW2c^3jrBD5_|X}SL}CZlCRkOn2)E4<o8U4JzOPwz@XIgdCBFt=yb|&k!K$J~xNi;|Hpi+KqJ_s5Z4SvHymlM='
    'Uj&P)XrUomkc<R@i~c3h^a^a+bIRd2S$$4dBQ||MR=k5%RW6j{0`xp&Z9qaGnzfJyJ&b@3tUa)*bP*n$4oYA-B}abq;apzYVf=v<'
    '>*rzK_hUsnSXJpl{WNv3xk_=-8%>@ur&EH*BYIHnK~<Ftix&()XNtv(?xn6g@4(*m)nV<(hiOATOjWuFC(ei})_v@L2VUo@3Fsei'
    '4E(vi^nR>x2dk=GSOyop^Kwf_*s#;IE0#Z!VI5ZgidEGvEVK*18H3eWsJG2B4D3UPygI5K>Md?iZ>cI5;a1iljF^_{EyCO**U4Y5'
    'KVa^2Sp6$jRk#Q%&SCx((aJYESj{WY|C7=<thTV?s#}cEEv!>c0WaEZ+n5%Q4MTL718WbgDqMser(c3&o|^lZoyMF!B|x$BU~OVm'
    'b&Ig$Y<-TN>b)Huk(t_zE}YwewFy=gE<y(M>^6(TtLA6`O802aWb3fn!|JngLE*<4O=a@n5=5GH{Fu!$JgD}d;)*JDi|Bk_7O*JR'
    'tY_10vvJ<5U96<Cir9jX!|Ho6e~)<CJT|e>535bAw6Y4_LdV=~e=l5kBsgbzdYcWTht&>NR#}B`!72%25wmA(A2{3i12X$T^{-Hk'
    'f~sd^(fZupywELTlq67z{D~ov!|LC#l2%!v5P-GZOtE;cqNN?%Tp@e{_D!gIMHM%jTErHsCTW7X;@PY>>W9@PR!~`mYN6~u`N}rX'
    'zg5a8uwzV?98`NyVMXPEMeLoG<#Jl`Pn{z@V=zQL@}Sy*s!A4+frWpo4{E}KR}+(VwS#I8sw!DPG)H^T(ATCubXi(7|A0BrbXe_R'
    'RV53Ez=Cm3H7;!Eh_3M~ZRucgSnXg{C5y1)1Z`uzNi#}UoWDf}8lBu~6RRp&KvY98&P}9Ni%W2jPJJnU=Q&uLU{%oqBevk4y@9qE'
    'JRq4XUt*b@!4U)Zz|yK(gcqkrfeh=;7|<CAdPvB2)WBV=s%(KAgN;0gP*(+fKTZitI*dQCOX<M+SFoyXfuUOr!7L-*>lV_u<?hoM'
    '6QOenZemr1i*Vxv4hZXso>(fJ+Qwi#Vf!vtRk?`vXukxpXW(#u2XebKCPq5a?Iu)Jxu8%kvQ2tD+wF&G3WC{tcBG3<u&Q)HSrY+~'
    '^&QB~aNdEjTK(LCn^;xpBHTFr&&gFvEA0YNbDvySk1<Ycv58fcE-0i67HI2b-TsC8O_@ueJ$@gjEV2n!l`dkewue#ia<ecoO3!nJ'
    'Ek=tEt1YaoDi>kO=~*sbk;CE=oX#=+Q|55qVYP!*RW3M`3pLAb_WPhcZppc#Od!iAht&>NRk+}2<%|ie<!TC&(R)rFco=xv*|)K('
    '!bLc9`rk8~V^%HF(q^}(|3^OA@FrGOxR8h~Y>G_x$BO-P=ZhBeBu5Y2g{le{k|U^+*V}0K=gMRK8Gra^<#x{1CRkOtkcch9F(KRk'
    '?pgIKX75edP8_(2RTVBIg$vc3vbRM6>*{3+8)G<e;3ig8xR5Da;Dc34?B9Hbihda?pUUtiR#msqh%KC#MjVKX_ljq8ZfET$WZ#2o'
    'RM{eIIq||U7WJzpnO+%UA7@Tv7pp2;ge_+`o>nCHGCNBXm@&JxbF4O@s;Y(N04l0J2-)rGxA1h?U$ygGZGu&0i*V&cMNUV$#kp!@'
    'E)-Gi`TJg))ncJY2aWY9nJ4<gyPMTd3q|Mwt14TBEhpgI7q?1<JzdUCwG24MLA3`}l`SkMP={y3aR<^ux`mu;9R3C5aQI8ZS}aw1'
    'GA#NUtSVcCD`%{X%NqW@Y@yL`JLbXC28Y!iR#mnLS5EXBt}iOWEnObNjIN2iF{Tin{kHduQPK3tnD_yvs#rv;nLkzVz~xcfWpwM`'
    'E*(;PNL9UJgi(7~(Jka!gEY(P4LZN^fr8LHIhG6!&*`5`NtTrSU{p#TPf(vtW~+g2iWNR7tq+3!zNo0_=l^*!D|^7IN);o+37yvX'
    'iz<jt^O2Yh(U*6M`MjM33o7OKNBWP|M^ok9sJNi&_yARvDm-QPSVCBq#?hnM%(*%$*`PN?WntjaJ#+NS-v6EM_GDD!0jerggbOG6'
    'Gn=SZ&6{}Ha7vf<^q}ICF@F9q{~A?@V=-kb9Q$6smy|sq1&{8(&oTw42PX`k<5sea{jy^3oI2xsf*StEq7i!b-kYB;c`hk@M2ah^'
    'Xs#G7gQ7)UarnKLwr75A@Y9`bACS5glzRI>g6w@9F*v)MRMnHXJs>5ORKygl%^t*m>iKfMR2|5kJ`?iGsQL%6hlR*JnhuD^b79G&'
    's6s2O&?x)@OjwI@y+?pEuJE_2_jhFl9{>Y-yAIcrQTe%Eqq2%BbP5Ci)XOOYzu9w{@Q4}6r@JpbKy_QG>i&+Y`sYQ@H=?DEceAQL'
    '&4352Mq$-wTp`Jd7G=d)I2=`C5_`J)(l4`$QE2x6t2Fd**Reg76+Oy<d#kKMuK?@J>lwy4eLPzMV<*D9!r~7i&kdtRD;nl>JQo%`'
    'c&~b|up+pK+U|o(Fg7!$u<p4dzAGyEVChC=f{J0t{9IJ{APeqLQH4_$`2U;B(ib5+>69!+Lw;9Q`tez>P<bvYrU$67q6)zRM_bHT'
    '5l0m+oS9z0)b^Jt*@vGZur>fF{_*-;O!DYNRkb4QHxXmm>3Vk2VsU1kDD$u#-WAq}i3;f+PYm<FI-Y#}*`t@L3KozSS%b1=^Zun8'
    'o;uFN%41A>*Xs|OUw<^1SQY%=I(#mw@gTG6Rj~;FP4pjURzdDN%ZAe$lNG&fQ~~{i0jW1=v1XU-xpyG<2kKI<f<^dm5_7Uzn6Qg9'
    'Ni#FQE8)cxtJ|ft5Pz%xQ~&d1!ERJ#D=j~a)vI6uDOhw!3*tq?rVgWNyX3#Ozbh;JKxzn4)$zCLxuEbt#lBay0<*8eFb8sIasM&S'
    '!9|ZR1>crbYzMf6^;lD5_*_`@K(2r)SA_LuIQhWZ*jqdg!MX2Vo^O--%dGh0v$k&ZTvXB@psHd;IB%jqR<jO#AFgA|+-mUfOyAxY'
    'mF3@}fI|lGb5ZF7!4|4m5x$%LR>_0iE~8(oo||RA%zKiOkI&6ztv{EP@dHv-s|eT4s27HL#gIk6W-Q#y+zVP3li!n-e*93?!Av{~'
    'd@ig<nH5yUB5XI~7L*(tSSS`UEmDVn(*CxOm3>fr;L%447xq+C{_n~IR8_AC+fB@>eC&3)4FWw0_~G_AK>FLBtKmOgj`IO34|(n#'
    'D33BLsDeefZXz0IlkNRuCD%Gy_SX>cO-W(<K(J2->&eIJQEmlQuZU2c#2T(E!P+5Ltb1Of{pn8mzs#zK?CBo?)DuMX&-7eY^ymt#'
    'f<>5adb~bzCHEp;_YXa_B_(Qa%L+cAe^LVFxu_oXEkTtm!g3S+usrBnG7m`7J+NW4M!hR4{OAK&J^CB@YCSg%mPd_CSQU$~+~gSC'
    'X-maI$D9QoXknTEuB_<eztVcN%{>>@c%ZIhRV>1B)9<vF&VaWeV4SPjWE;JnZ+ohu{kxGtw+4Q^o(f7Icp)fOy&~*3JzAfJFuJHN'
    '`iCBK87-o`FR9UOpju;%k=f^>>JK77tb&Ed6CD4ZC5PJx5F-<5Q-Ig@zSp0882B}g{%>b&je2eX=uyK1R8_7BvrP|NXY&Lu)QW3v'
    '#}g8zca<wpKScarPBkX+1mwA-=z+A3RjmlGO*HDbJXHS2&MB<!e}KGi{AYgO6M3xev8<sz`Tl#<T!vMw2&YYS3ek|+^)~)ve~hOk'
    'U}RWtTd~6N@#F665}Q$;99PH#bsei*kz?niIS{wXtNLZ*J*A5`Wkt>Z$zk8tuMwUnvwE0URk<RpHVI_b4*UJNQk(u}9t$et`?4Bk'
    'e(|}e>_O2Ut6ZU32Vu+s0$kjIG;n$XnuqJ}DlJj}xF+ncar-BqtMMQTq^ehF>J^^4w{<(9zr=Gzh=0Lg?^+F}{OLaR!#o$3Jt|vK'
    'RV>16)6-ymV2nDVa*$qh@V1RhgddH>pvAKI7(#lkwiK=mYf@FR2(L}Q0F{=FOAH<6JP4@i`;zi_*ZgBZOl{(GA1i|gf(2E<LWf7|'
    '2*gW|xjsTH%(%j@dVF<1z<XBo(fm7%5-F%t-t9C}QQ?EOC8~l&*lZGWj+rjJhPgu@DQblE*US52tyod{uP4KLbOBb)B8)cu66CcI'
    '=%1SBsJ#am+5Ggjtk?*en12rT41}Kxiyl-$sHzs0T}ziBf%hlNk5rrUJA-h%F|6s{^TW#L$*dlTfmB6{FxvE^H47cGXx^N%tqF_Z'
    'J70o7T5JKd_a|S1^ytN^qJ`zCDdf(Haqo@A5nyB{EQ<Z*7liyZu&Qj4hMPlr_B^km3qMZN8Wn);u-d|kt7;LpoBpM09;`(NJIzKj'
    '|7H>SpxT3~Y8K(TNm0MgVFOuIuda<R5w|NISbJbq*<z$@;iqe9Jm;+1WaVi;VLz<)u&T<1M+^R${HI^FFKJ`UN$&<83Bn02Hlg~g'
    'Tu3@^avL)s$NMmlFXv6_Z?J>*uVKZNm0x~<2Dl6@w26$Y8^_G|IIK3YlFBM#3;#YMwz!`;<xbagC1$VhzW&#+(#k4y3&gavZHqac'
    '(WhqELTNwm)h1R}S%q%FG+P4~VW5moR}a&Bi??sZY7|yIV+(6-X}#osEM{ooA^Ujw9Z7!+DrprJ`V|Nxx7VAkdIQIdETW(NZTFf!'
    'ot1u#Rj;ffv>3UwC3;>KcVHu$6@1{<53D_~pu&o|(>_?!2JpVyf_u+uV^@#*`LNo<3M(s*!ej2EEoJuT%uUDqxsjU=t39l$ZV?`w'
    '98I?&G3lx#(sQ0W`nASkwTV^LEg)Pt{ECZnbst=qr_h-#xA5BH6I^U!RdtK-;A9|zb*a&RlrhKTRE-X+O{}VJ5gwevHm9}6qMMy<'
    '{uY!@@4Geq-C$MWB3wAxF|_rR2$1&a!tUSyfWvAJt14WC5ogTS({j5(u5``^%fvtIxdivH(yCm952p-<@jztiecqFvJ)bLZ7pf{;'
    'gae%ifwNN+7S3k#3hZEl%?_(Qtg3DiMw~RjVCk94$k*odY<@o)(<W9`x9}g_{Oi&OwMGx^=Q19^-qvkQ1w#ux>fKq5r?INKMfm4Q'
    '<LD*xJa-}9j?3PWJ!)UurXE;-4ptQ|C_TP$dJW&l7To<S&8Z%v2Oi%})8D`U{QCZ^VrKVAtg3JkUYxO^CTFnZJ{eX@SFK@8m~Ku7'
    ')(%)zxd=B-8U76MYKeMG1I~hDZ>D1z34Z>TC5FJ?9aVoCs;XOr87Gtf`=73XD@9OeZ8%&<n;ceKSXJF3+&H7sm~FKSJ0_3XbD}P{'
    'IMOm|SN|#;?T=sIpA|fdl~vt>aoVxdg9btQk9~fpwR!YG^~X?E-696;*@DFKwL-dHD-An0IH<Ous<H*=Mh)&6;kElHkm%}XkDOQ!'
    'sx7FhYT;+ZL%@;d`L|VJXmRVwG<l>4iy059KZdG`7U9P!o&}|K;G%TJxkbH)a0_%uZ6Q?^i}2&*Hc!lIdkEpx8|cBhoKv-lRaGp)'
    'kkb>~G{#><fCSf0o@|po5#SzFRj&v;PWAh5QM9tl*$4cd9Lob`0UiGH>-$r}r%|d>MHq2PN1ChQ?$ckgD466hBUr=3Y746>RfH2~'
    'h84L!?*5@-n>`jC<_~{ReLt#Em5T7;^x&N{`lD6&C(|>P(UcCVEvTwe5iXn(I>cCZZ}R29<QE{PZQ(ttPld(!72lr~J&jeBDl{#='
    '$z<ISNCuXZj3`Dt4y!$^s#c*9S;Qdey3tlI%fRT}T(4w2sQw(Ps#Sy)r>b(+Kw$A)jaZP?a@zs618WnkDpy!qd=E*Z;rlyK;8JUQ'
    '{1+ZpdstPuBAqxp%@}W?teZh@n8PlxZiYFm{v4~SSA-L%fwZ~r`MzQ`EXRnu_<gzRg~5-85XbcT^)EohPeWBji!kC88J4C#_r*nq'
    't9esQU{$d?sQw_Ts#%OIgY=Xv7Bi>LYA!ovzY2v9stu_8`>v=wu<%F_9yq3k^W0RI2J}^f8Gdw#9<(37{sm1-@-$Rcvj`VXVVtAC'
    'i@3sbvXnIjH6%KywxFt#MR;&ZW9jRPl^;F226%vge-~959D#R4MLSSc#Uk1;d6>-$@Gb<zHW25Umxh8Xqb&UR{8T||_U{hMo(Ai)'
    'W}#`qIr_-1rA2B5exC9Q^l)*xnGruEmOb?U{`&T;7@o$8E32r6c#``lUT2<5D@YAisTF;+_lNVmTTd#Sh$V<~($^OCkHOPGFj|#x'
    '#pdVDQ*RDR`qMyZ1(gEDn0xzVB~YZ86hp&o%Tx3zjEG<5{aHQlP+4Ua3WeDe{_dyX1jh^{92Kl=vH0V2lTz*hd~;OVo`z}^RN=Vk'
    'cj7dzxWE6T&jHjX{z3bS)aQxqJb>>`ik?NPw@NAm3iD8!mcHv0W=ttJL0f{-O6vVVJ?-}Ey@HCl(n0vBr_3LoIZ&W&)lm@(Vm=R*'
    'zJE-iPs^e@R8rBTiH6zQw}{M5=VW~piWhT=^3hxD)cnt{@1OA6)846$6_tk+3WN{pzS;)P@NdYnPbs`R)f)Wx_5JTZds?L4t2Plv'
    'n>LUioGQ8uZ2O1?v_onOsj5ssOq7ZWu4^7T`0A)7ZHLqzQdOo1lTGy>JUPfJOothEfUaLER1}c^{Q8fiN~w`voYX%y)bb8eRi_Ax'
    'P4j=X2d$f4995nv>b88Qq>!@r2PIDfRYi)h*YwbD&Ppw$zbIhM;4kEKT(PPy6@S5@vZt}CDn+zVXzW;Di+?g?dgANMlkQ2CFx|%6'
    'lN#+=q@b!4VXirv^wg6UJ?aA+&SFQ)U4VC}{Ch5yDgFjjeift9k6*u=6#OEo_@aN0RFx^hTT?a7d+y@Cel=!Ve#{l5PqR7x>v(%e'
    '{4_{ap$Jn=^Ov93xs-Z(JlE_9!fKFSKgu?u_RnHcQE>USN$Ee4k{zU~K!FiX_<d$;Aw5dGbAAU=SG%k9<8vyd_V%F0(?C^uBK$N5'
    '#0W=>a?oPrjoWN>wkZDIld_NYFN(hJg9;iv4OP`A!cEhV+Jn`7VzYg=b-)yt?@2*bDE^UD`Ksl$N&RDSN_UW|`b2tZItX6(xENhw'
    'UwqV6Pr7~7b|iXxQu;JfRi6kW%>hK=J7(kl_LIv}J#8FiMXL6V$-9&4{b{7CKoLHg>Zc@3<Q9wIVog;_?c%BIDxw%4wL10Bx8LVV'
    'W7F3r^^doo?I2a<iTpR8%`{5$i=o6}s<Yh%ZM9l#d=zn3@a;+Ivq)i;C&EK>u(7PCNBi6_YccMzJW&M`$h&ib_B2jaoCpI=i-DXt'
    'Q47J(QRcF~9=95<Nc{`J@0~#5(?C^$g0qFf|9h?Xd2}#Kh@F<Z33{uE67{zy^}MUE3KSdyMch&;P^?6B)SLqLfC9QKS^cy~;@w%{'
    '^RB<DQH0OcFb_p@EPhd&YLrXs+la{>U9u`xfBN>Qo_7IOjY6{EVYBYEn4CAdP$A=3?Z52(rx~}mNA<iLu!<CtL4{%l6?~6MapZQE'
    'EPt$4Ydfk>U!(u}_BS9s?*^<gMfhjN&puf#MhNEmmhRC4uV#E5)lU;{Z;$GEH(=E%!bCG}z`1;CQTf2>btT~)vLrSBt_$#=M|`)C'
    'syc<HPSGJfx9_L1$AD`wi~sJ`^#>oV3{k!RnX+e*qN-DbgQh36GpbljP|<&Oz7ei=eK@M3((*qYD*PzLkuRhAJnOChLyAk~`FBb2'
    'KdQ9jUy$nmL(0BJswx$lLB&*HS1;8NP6JK<`+I&p3skjtNZ%b5J`GjXD#Ab0JgMb%InfDmM(QlwmytSrJiPwi3Y0z#RFx{iJkv9('
    'd8QVP>1IvB-oT;&ybAvCrvcJ82PMzD{Hji2X>DaVC*v0~Z7etnPTc#-k=8*yo*#WHQv5VhRi+5vOp9*w<$`)?+%>1R!wG8z%EP+S'
    'OYt3`>VUp{Gpc_c$^JK}@OM#FnIdd6ZKT!g@|o(xbVeO*tSnBQ6_qIZ&$Ig9`}KQK{qm{O-$hkb3X9@=?=!HwFVTz4-8tbMVzMMv'
    's#L9i1F4?{t8XP`zj~-b5dS?=Rip^hOk<pD-cS~~?{%segm}e@KcK|v^v7Az$IolJzn2yLBC80qe~(p_DMp%GDbCeki|{Yp8M2#9'
    '307W=&Z_P?`tvs+fBZb8|GlinFS6p_jVrhc6$3p<>Z7GO2;HA6li5-+8m^YIbom#}w7*A6K7I~dzLnJY)$CV(n^YAl!ZmYf{HwEU'
    '(H!we&E<(ah?l6Up#J9%)lYwUd^allMO6B`7a&)mV&n{|$9GxUy$#koTpPsDJe!dvs(;g@`8z_b{nWSdy{Pb?Th#xBP;0*%S8$ao'
    'Mif8dtVLjPFR6Pxe;qs{)Ky{?6U0T-{^O{|Pxa+*XZ5S~-|^e5KC2ZLrkN2;=_3!3A7-=-#%eELkAlkQ!M8tN{iXKNz8Oyc#e<Zi'
    'W4{N7E1al!sLiQ(y%#9Dq(EUY^s#a@?fTQxW8Xk$t3rXDf00%HZB|lQWz8czDWdIM?qP{tl|)wQtD1*>{0m?E-xwmG{*$oy-Irb;'
    'zcxi6@Y}J3R#xf(dV@<#zki@~8l3v^8nTQf>_dvr0sL`R@UbyK$A506{Wqk>?~<}gYNVLn(tvwW^N=YGc2aJuNMih}Wf8vzs=AZ#'
    'ccc2nAu9Q8HHk+-)pLe44$WFk|ItX+gamZSsy340pPnIgR@L+3yIEoXwIR~<xcMzs(kd(Ts4rIWjkQ<0;mmveL1SB4QaCL8nZ%#9'
    'J0PsC7y8|-e$kUhsQoUhURgy*fw5%t{!Gyf0EZGXtu~Rik2;)#-+-w~z~6nQsQ)Ub-_?=?DkshMB){=jc4U745oi2|+Q+J#=&Y(4'
    'H@rV9eHtsQtfKNUa&G4~r@9u#MsBs5^@aXXMDE6KkBRANn5sAdDNbm5rt9*Rnq9P>FE4F_0jeAAANNe(j!J%Up~oLURi%os&Ga~p'
    'bKG|!&-#}-r|kWFtyU2@s>j2L??g)b(@0gJA|(p)*MGS&Tfzq>V?PfP5LOo;{!~cBw`cXM4$pspRTU~AdMOMk)8B>3U+gr`nct!Q'
    '1g&mB`rsnazCWz-JXlq$h;3eBhn~&n_dBzth0n`urx>_e><0Xk=fV548c$<Y#R`nb0!E+9rS4JL6<IKK84Ixj<bP^??~RJu(@<&E'
    'DloJPKQQoUnAD=RDu#Fvv%a=q_0!Jmw}<u5ULspyRj~ruVNvHSN3B|F<!TW0-wmtxN<aRE)#*E-0)85*s#k=K=IDc$(z-y?x!Kb1'
    'LdDgovY)1@-k#JykJE1<Rn-d2z{2CYvCgghqMXz8W~ysc|6)Y(tx!Ghv8sxNhY8%%%^-Sa?;opymL_t3{`|6B#em}jfBvf992U{j'
    'U{%e6FoQtK$Ld}R>|Jo$-OK3BmaP7#p#Eo<X76XEzqT~{53s7LMVM(0YnoQVi|1-!dLOi~{S#cWD$CyF38?<)oHo(;bEv*Hr>#}A'
    'py;l^-XNvG#T__W9=$<+3+nm~EQMgww}-{;X|SqjL9@YPo*KO`G5V__b2m%$Z5~#8SXI#?@}EZ63fYBn>;Ij}vJiFXht(EVR#gj*'
    '84y2Jb63Ru9Y{YGUV^91a9HhORYi+1)*ODS`qEfLz~jKVbE1pjVYP`>RV}9VGfl_FzQ~0$1<S?fWaquw#H#8R;qTJgwf614eceCU'
    '?B2A9?@!3Sg;bR-BxH*S!yoi+`ftnZijBw0toxw)gQ%)#k=~lrEET4E(So9UpFtq{!*)>ZK~*&i;q0>I3>p@vN^RO;1+dkI(u3*`'
    'qN<vOgl6G!;G^cv29UBX`ZH_r+NetR{CFi)@-<Xdv5?qe-eM}QrD}W^RRHEtV{eH1+Ms?KNO>Jj{54Kht<X>_&_DE8c5e#m)x4>u'
    '%jw0JK=pY@xW5uB`Vy;A)d~%@B7T|K?V@f^m%kEAgK=1GVO7-%O$$aqSdQ)p;2d6TehVt#LA3={6)QBv3h;v!Pu0D)j&>D7*fU|X'
    'gK7(^s#R!c73zs?daKK#fn6y(r<qiGKdiQ}s%nMB+If%eGOxOi0Q+cd8bLFvt?P^OlL6+tqq3)=s%nKrV9};!_reHYa87tfo(%Rt'
    'uRW~(6ssy%SoUtl2*I+vNIbA-OphKbj)Q6osw!86$7UaB^xy`B*}Lgqt{hHVI908}BC24vYt7sJnF^<z>92$jboGPkPob)E#fTC}'
    'N6hKOz9?4B9l&j%$L!~!+JVaBxQfby3h*zV9`r-YHN1m3w4V!b52`9wj1VhWlop+3Jl9~h76AwzOj|Hjp<>LIO-aYb1*-Ve8KuL>'
    'S^a}*1F9-ij1(%!W(a;CQFzLGHI_Z<t$jMFpEsMm4k`Q^sn0US2y0ChYnbaLZd(9V{d$}`O#zy~LA3=HS5!GVg-ss#Va0KxIftr7'
    'cvx*=C6!f#6wy?+T-YZeBV|fOF}8C^Z6T$Vlt&fdKR>RYD*r}o=Dz;cjdKC+L1h(HG*uXOq-CoT+BF5z8*@AN-yT$>pz1w6VIagm'
    'UG!Cu%r(FbJQ05NVEu1l)mvp1N=0nfUK$OHf9!b$0-AyxR9jH>iYmg2aBt*wrOJQP>?<QQd{C@Eto{@$sH{S+kYNv8Mo8PWg^U11'
    '52`Jwu%e2<qBqR)?)_uM;vX=#!C~4@9#R`fRk<R}Ha)0FReiLTUOP|KKo39qVYP=<)hi(43dg|NSjMYb{S<6Tnc8`*_OPmY1w>$x'
    'g9D+P-`{{5UHWN>WrI!@ya!bkEMN*2YKz;?X8)f>srXRZL#b*N;j<Y7II=V@<8e)uHig{(a!73;Ri%n>*$h=Mby!=>Dq5QvGUbnO'
    'NPRyks6s`UY{nwyY!@S1^l4t+A`Pvdcw!H#DpZ8UCSZ(kuXdLDAB!#X<4!vC+<$veRh<Gur|^dg)~dQN%_yC@b3euUJ*cWq5gwaB'
    'HZYAF@1qKUsXAG1J9uqWWz`wI5-Ru_swz~3$0o*qjb*b^D7n*ST}(v|sx7FhP7w~9fJio%_}w2XPPf1io(32mR$EwAoq{5+XmE{8'
    '-NzM!VV*qWa|7-|RdtGR*aW}VGH5VanZV3;B@}!4a{XV!s;U(ffklpNFJCJroNIv*L|?sM>90Xm<%;mv3_J9Etrp=rP8TeX*$M5S'
    '+JXwJUJ>@1(c!FX69gij<BUc)>^xR`P*uHxBd+LZjn~qW4C!;zM5m^Q)gD$=un2ce=$cvZLR;ccb<<r_8|X=`_MobQ1!u>mr481?'
    'u}ldu_uPl%+vuR$f~pD@;jZa9u-barSS*f{?1OIh>XW5^4XLVEL}=GNXpje<WsY*j6)Ylf-l{#Us$LQ9njmfTwYbhU-zvX9J+3&>'
    '`W{qOuLyTdRFDiS7D;dvf~Tet%e2q2+QX{q6=AOlZSY)vzKAR41gar9+Y<urK~?pNaM*;HTjZ4$zL4Y0Efj#(Ph7Eu6jixGQ?Bqs'
    'wHyQSFLkQ#2=eN4aDNS`DprKKrvEv}YZn-{F?~?GqP}_`%wL15Y89FuSh0aI;)VOPDpr8lEG@FY$$<B;s$xaBYx<>T*<Epe0V-mq'
    'UY6hQIaYg6Rk0$>HF3bSS6`%6op>%!N3?bxt39ZyT4A$kb2>q<>Q$H>xE-i>JsI#GR8_19Yb*b_*^jjF)->hIULCm6DcJ8pRmF<%'
    '*7Tny_GtN^wy4;nW?W&t4bziX?O|2LityLOxxs$5Zd+vFoG>tGj7M17!>XzkBfBf2BEMW*<=@#v#od#!6RvMT#Z|5dgH2+jwyrG-'
    'T>ip6)^490a1W{~ScJi*$AEoyx!h+~iql~vPSrV7dq`EiVr0{%pOdy+1`=OP_H1_Q=la`3s;U)ZwrpPObM9ZNaXHLIf9>gl*@Wto'
    'T0!k2J7*o1t&Q-t&uUfdV(fHYsy(Q<q6)R5^9&dkZ7_aMYM#8&=l18rY6B~&tfCC!LBqlbL4UMsqJ%H%!L$XFR!kvPcxnslp~di@'
    'ix!yJnRyP>CQ?>OjXY4p9)#(=&&CVY0sZKIdr*ynD*QE3V0O(e4%N7}Hk<LV6H;tJC9R?goq`y%5Vb0RTsvDD#5&}IY744fQE@n|'
    'gV=+ds=-_sOug~B|MrlAN-A^;xttIdVftt(4|<r6C!^X#3M;7yDh8wS^7^ykpDj!Ez~eN)Y(iCaiZIutDDI|d_kC977vPK$wc|Nf'
    'yI57B0_HN1KG)0Mmp|<CSI<R@?dX4dSXH4SChO6^{Tu?h?W@pitj#VN^ACHjz&)s{SOF<ksExH3zUrl_V-q<y;3ia6tq6xrY@^LC'
    '=*0~<+K<i7EawK?#Hz{_m=zGhu^uk9L3VA~Im4x!NNH6o!ei5)sYh;r{=sKl5q7Gl2YL^wDpp{YKWwfTTLcxo&zH(U=?N<Kkg8%u'
    'IBXI$u=Oh7;bEWVw4&$A{XA5AP*t@e95%^G<)NbcQx%PaX=^s5pJLS>QdO)7gH4YpkROa|JEF_CR<*-9ReM-fu_7HdV_aZqgr%Kl'
    'waQ~ZJ?2WgP*trW3^u9zSC)eX_pCx!Pk%WgYIuqudstPmA{;ik8Q`oME{tFxON$+2*UrhS_OPmIML2Adj(NW=3RXfnv+ch`aDPy3'
    'K~=>H&f(G-G-S`&?Q<pB2h&>Y!#pF4TUc3@E5c%vJp_N?2W|ZtSqzgOILB%as;XCn#U}SgSqQ%GF7w2yW7ez4PG{{dR8_DDhfNfi'
    'N$!1ntw!r}R;)Y(JPvcaSXIS>Gq9j;%Zimoqk2|K{6E8~SZzX8C5y1wWWV5GZCo2(Fk^=6<CvChB2@*8u-J_0dK{~sb!|LND_Xav'
    'dXnHhsH$KQ4x3@ym@9$rQKkPu=E)&foTq9Js;XB+@XgY=(bBif^fE(Lly+uz_K>P-MfhuyKSQ`|+SCsIyXKZ{ITMS!P*t@e{53sg'
    'vi7LBh+}R<iguWjR&7Bws#p;Qo6G}qF^jC)U}gXq9ZP3qdlRZER)oc-qteu3VWHQ?uEp72>)+rBEB3IeYDIW#GB#es=?!O_jCIQG'
    'E{D<vN>!%_cTLZyAbp_kQ9%^-Gfr=@68NCnf~x8i;jSq`VBG{gVpu14A)!$_ci$dVRj3GeO*Cw{R{szwhZ=kLFON@1u?JNZD#Be;'
    '!^n}>-bzNS8T}a{wNtCw!>TG3VXx^yh5v&`{*&%*K$Ogo8>+#1tTv&lQbo9HN|U~*QuVnE*wfzWm3k;`pj1_gF-J>VR20pYEfqM&'
    '3a}kiJ5WUgS5bLLA=+~SWI0-|G&hiCQU9RYf~u+%;j2jws;>nl^#E{gi<6FcmbPqSRaJ^`)@0S`N3nRUu0=?Vr4yd&#~xHwsu(F%'
    '5c6^dPI0bv8Kco?cuv(GRG+1ap4OTY?Mh?asT2#F=h{-G28Y!iR$N(yw;bfW)IU@5ZwpgU>b?0VJpB}VC`qLhiLW)zjA;?6lT8PF'
    '@bh%UzfGvLq6(G52I+FOjyAar5L6K4pxT4VDyk7Rs=2a$-0uKU1em)gD5iD}sx7ESK^4}TaRII^VAO-ZtMW^DMA|K&daIyPs2D^K'
    'nw0vSRP=<*fA1O7*@LQAR321F=l^5x474N5QEdA~<0T0Y=>LE00lGP3?7E67hXXs`qIy<NPj^<9Blbu-I)VdP0+!r&R1H(<9_J&T'
    'O<bfBNR3F!vMMGC;&E6N<G7sKiFHmTk%CCdB!yT#%Q00B2402FdP&B5`prQVT?z}<R6Vz|Dh@U3)LkI@bTO_~m4hmp6qc)LC)HN!'
    'hocJT>R$H;9c!>+602xc3|AEvY<29?8Sp2rdv>n8?j%&vs<2#5p0FoT!M#sAR^M}VrckSy^+~LvS7E`LQ_re?Ou^?dc1>9ug#J?%'
    'ssvQgtFT;6n-+0?t7jw?AAVRh;Of)PK{cXV0Wj^Vq#yYdfl8_>7`9v6vHEn9P(`=Gk~L|L2YreR*nCcABNZ4_sB%z6yTYP1SuTBd'
    'e^!wmReSt)n{Ad_&p{Ra3X9ex3%J5lUL&J*3IEvut5+upRrD(?Ta)M5RTb-?0Iz|i*rZC8l1N3n!jd&fFH7FsaUTK5mS%B9mEmZ0'
    '2hL#??Fx(5)ZNeKd>jJ10l>%4b`(|jbyVe$if)AkYck{VUPU~(6_%^y<HfL1C3YvVif)AkYnrboCK}dZ1H3|&#J-^t>p7^RTVcVP'
    '(|Ul9iQSfxFg#aNMLj_oODU)zx)nZHySH_20R}*5cnxe3sSPj(Rdg#XS94Ci)~-{pA_MO(xHW;XTC0*!MYqBV)oJVin0{{pvYuz#'
    'ueHY1S$E(ZR?)5?*A;VD0UoUN)%K&76=PHrND``OS6Hs5*!*lS1iWWOHVercb_PgQ0w%GFhJ__-at9PVhpVefoVx^L4XA5y602xf'
    'SZkHLfakI$t1okRy^lTpxlkpbijIYu<}##>2mJ+B0=--1xD6~7n8PYM78b6_jMHNP?~Oq0duF;GZXKvjs~l9(v9NGW6&7=z0=rdL'
    '>}haoS~YZ?RVkz}8Ws#wrNtWO3A3ypdGhYr9=!&EB%z9ig=K46$bCOQ1QYDg3RM^WvlpoZQqixlWKI2{-HkoWf+K0m7nKZ~v0bQA'
    'P({1K;x#A2Y{&94o`Z*9fjh{YWmn{&igty?Yuejx$47?k^b%bCgsZA6a!5t9!qPRTkQ)()ECiow;Gt!RRClTzQqijLa7_!&q*Jyg'
    'TMK^M0}BS#D{vC3XjNFYrmV$oRh+KBS0@O_sC>CBRMDxhY)!rVKI&V<DX<4tVWU+WU=FJ2R9L_!ThG*C^XnT^Ys@Hhqe{U<(WY21'
    '=V}0Qo-0Av^<TdGsr;)XtRz;^r?7NQ7Vg_HKfC~wKLPgv(y9nB2~{*IEL?NqWH?S%WNACGQ5H(|Y$uV5K80m#uHMHuRx61IK<^k-'
    '<yN=f98~Xpis9LsB7Kbq>JR!(f@xFt%6g~DK}8XjRaMB`)k<R?3d^pJ9gEDb?$RVyGqUn}O+vd=-4E<zzWPA)?hB)80FzL0L}gZm'
    '5c+JzNv9&fU0b_4Q7x+^RuWlxRfX))IrpfT)_wp+?xg^8K&=4EYcORDIE?}sHYRfyYpPVN-7f{yY!Q@+3Th0;$+BYkF~0Zdt%@pI'
    'JqcBhs8myk*-Gauz^VV^_g^BCdi_m8H6kk06kV<{NB2_QnoIrtgJF*I21qi{sxbDi`1iBoA7cfP)v{$Z<Pcb92{68-tgrFGw7%l>'
    'ZJeT0VfmUWz1!mk5V$6`C<0ssD+gBeDu#D}4B%t72n&kGkBr`+k3yA$Dq0m5u*qxRY}aAFnAcu-aP4Y1d%?<q70n8Z*j#|{NLUcE'
    '&jazS;aMkE60GP}SjMId>h8uK67lD}=2q#~hqVhPv5JO;g>33Qc<dU#`^QDzsC*&_6{{Rp96bwAaiKqZ_rlqJlI>R?2re#pkp?Te'
    '7C_TR0(;SByL$`POV!}z4in4{jf+(dt7uzz&?fcab3yKl))$T43p@$NgRJ#p^=+)8Z2>MWW^WIMVoLA(#Ig#`2X&hT>)T*O@4~V+'
    '^%HeBCQjDHpUQRAKVlZE99Ge~u(ZvkI^0e4LoVQ)SC>`OZIkOhoCGVH7wB&y`lWe@<Ke`TtJI9Sl_i*jDq0t)E2cFPcbZNqwhY^i'
    'vQpbwk0MF1qIF?`o4m$Oal9`JWWGwxI?Y0rg(^B1)TdsV82j-BpRrwIx2-;U)CDVtl|<))K=(k@?~9H}-m2?&ER61dqn8lOf)$+$'
    '%iLT<;*D_!_jc{j;il4kmbOSj6^#pTU=-;-cjtat*&t0f4OL;xR_9d`t7u%18y9_dzTGb?{ap=q;M}7k_Cob-sG@CQnVTyAcAVUy'
    'imA=Mrx}>xUKUdlt7uzT=BBKjjdi#Q>leREE_z!qYF#C<imruaZZ4%XeNy^7v@+_w%q^x?Pa;{UqH8hTwLp+_kOk{0wM(d#>#BN-'
    'Bv#S1n7i$CPjopn$eC>cd-jA?eip18SkboNt_!OJ!*Te801J858JK3Qn{W=R6>STKDKLaQ&;st>@b#ZNaCp%f(~#1ENvxu4VZocK'
    '0((D?O0v`+-vi02+G`I?Vij!*=4pMu;#c9@E;-eJ)^=c}PHdBdi&YM*Xj_;ev1mI?hk@x<8?)!ss=_bPp2aHq7Sdg`({Wv?4qU>!'
    'n6pu@Jur(^^errYbG0GkT*kD(g69lpN0*YtDuY$@Erg~ub+ZpOxVv9jO&ac6PMT9z?SV<GqH!T+Tyzq7P#!p7w<n-=_f#!s602xj'
    'm>3s=xoaurePC9jeZdl4gHkS260B%kSP18$wojvbM{r$6yXdAVSjI&PtKOn<VM&}_?@Z#4zPIaYE?Qd%((1JbCZUSHg^30I`e7V;'
    'uk@el5|H-ztHP9lDOwiGXOIoqQPB6L7JY7Au}pwe?^PD8=vjCUr>+@1d+aLNx$nSyX8P%)s!tS)h4Nqd*P?3AqIz#xfaP%N_NSZN'
    'xzxe}6Fe}GAJ2<>J*sBKZ*cu8zZI2@dwUKQMO5ZksAS*E>aeP4=hSsD_o7gxpqdetiVCkZ!^6XsYW0r0@2YhiKU@@}yS5&#z5V(1'
    '*B(|N&$8mk%4#_KXkN!Y4#MI}qFamTJA}p3o7~*L&c6}W$Ur3#)iQs*E!qzqhX(wCey<w909dS2SgpXSo3I$L)%ZM}R^CBk&IWvk'
    '27C(DJ5cM_g5oDZ^%g<FoP*jvi0+$&1#$PsNw;@lEK&)idL(65g^+ju%BjRcFTNh`y~g^Ea+m`35Y&jE><wfL>Bos7qQ~3rw!%r@'
    'knZ@NgEa*GZJ>GvD2SlUs2KbwUj4o=&K8o3j<p}W3f1dTMWbR^yD0(kIebY41x~l!sqy-TRI@_$FHl9J!a_Lh8Opm&`~9L~M)KKK'
    'Dk$)WLB0Q*rT<z|^I1~Srx>OvV2L+`+)KZvtI9j_=M?Hvs8Ucxn_{S@=ndgm1JkW%uJ7=KW+NF3RSK$TQw)FpU~rv|2UWpaRj8yF'
    'Z}KTt39O<`0p1S6BFFMt(c{}QrG>jlEvp<<BN`Q8Ar@^7uSB}fAQlkV`QQ5C=kuuEKVVq2&|inDWuS^q1<+JU2Y599#sI!@Z^vq5'
    '&O()fDtZ-QUIlb~@K!$vT~=81BasxU6jafz0DJy`24eTBdsH3l^6SAc;;Oz-C7_CK1)5udQ??I#aw`B_ddv=lby(%FigpE>u8{7!'
    ';t=9v1AKONjL7KE+UoS_+x**5HO@d4{R)fST&wxmbYAzYfW|)1R0}>mg@WEd(7ea}>tCqW8)Tr0euc$u_H|7nyEg%CVC~q4l?uXV'
    'SJixLe;Xaw$FGMI9SSrhvFz>rDEjnN+t0$&Vifw*aM&Iu<<GC5NVRvR68-mqlFu7n`+iW-p0JQj6?|Jq+&2=y6$7(uTJW>`9mnyE'
    'pPzi+ziG5z2#P;}YU|rUK{O{kVRNC6n>ctA`oVU1w=5=nR-4NkzppTdzmAkmZ+ktd=uME-W=`mQuA<e?ycZqu(`~=&Eu29gb@_aI'
    'C)%fdy5Gi$J{3_k`F>8(n6PZkCc=E45UiJ6zd?1f5W43;sZ#oc;x8kGErV3_C1`oM3-7FudsUA5#rvAFAn2!2jkiBVN&f5SMfHE('
    '9QNz)Lp44hQ~dp?qAxMcmzZ64X568Tigy)->WcYv!Ao!O_Fnk)@z=5Hl))<c68Z;DVBCE-hbLdgJ#I`E_6|?8!ngON+WN%mT}Run'
    'cF5nx>Z6$g{^fl)t7uG2E2Lml1Xjv?aM5*9?H&Sb&psm>s<3S6??cryP(@?Hk~PP=ng|Z%&Y;KK)$Q@@lpQ1We+H8H6x+Xxlvo+-'
    '>q$kIVydpt3o-auxWl4?Yw$LSx6Bo(1X9taU@y+>`=sK7O@ZPAXNT3k%tDoc3ZqfMJ_@Ai_j;^NSJ^Q=8f%RPwf*Igibe&irWksm'
    '@+;G!g=+LmtM^I0aj{Ba70n7Zv%*G*9n((*d8eQ1qGYL6l|w3;6>OrS=^fbjzkB^R*4_XY3>-X)RSK)<R#={<#aXon?Kxfe3O@z('
    'N{nKaz$&^GVv$!W09~vf)H=5PG%tNL;$oG<D!LV7ZUv4#r2b^s{>ePBk(g3fs~lF*uCP2!;JLksQ_=GkQpRnKURSFeRMD>xlNPYr'
    '4oW&$OWZx6rvWZ*#VUtYbS#!R7OWcY!59$u<5<9ep9U2k`^RquMNfjF=vR1@rvBvXJYo^#nk#`8P*J2(NJYQGk~H<`X%ioG613%0'
    '`z8_$DO4$_qFu49I8!=#`~)A~KzQ)(rN{9gpf=Qu@!wBMeu`AIE3D_b&7l|D<D$Z~C-w;}!UszI$4Pyx1^6|j;J=Z2Z&siuX%3%('
    'biP!<{1eZEFdj717piYUMG+PFYyEmq{yZ6+rp<4DG_&;eq~1k4wtt8c{w1W~SyD5SvSLlVf|U5+RABGkO5ggzpFXID5O@6FUw<pA'
    '_ADxnsMz<vojrTl{$R(*Eqa_z_G(o9(?8(@sQ{|~6shrfQQ?0hC6UxJp}tl-SJon3HU+l!vzuT0)H(7iIrXPGt-z`8Cen3SO~m@G'
    'yF*KzXa7k`u-?LLZO|{p)W(yTW{a3CK+}@Z&z)}e<a_Bp{WoXb$L}w`^ZQZB+j~&h=c`ezFQFot&!XxP6<OAE>u22uo!gF(L(3@p'
    'QBpTualecc_h&hc$jQU5R`aO@0}lgqX5Uc<0|@(%lA^a6nnQjmDxzmmK}2Pb9#fXT``+(<swH+SX<wr~7)$kPQuaf$w6T93zZBFz'
    '1akjJP|=^T08MDPeDgcBlEPJ@&a~&c{2-{2toy!5YCJ6{qCGK8P@rdHHg7bzTT|F}d=hrT2T`Z*6AT1jp#FMHNf3B3EPE#U3as|D'
    'q=*iMMQC=QWtD&ZzDq}ssy4jzk6^VQX4PWr+?>CY6|SdcMYJd^LlY*9c5WzuHT>D5E449ymer6CEvLV9UFn_XSys`c7_KXT%z)*?'
    '9!OW|--0Rd!>su2|73;z`b$yGrzK$YDZtA#Ra%}aYN>5MNsp^04|)n-j%t2*`S`6{AU*3TaWpF|M6;uAg2xf{Lc+4=C@uj0VOI3{'
    'AMugZ|GoSI{1htujEepns%Tb#jWM^#?qiA4Li^M%EM$!S!#g0ptw)ynZ$Jf4ql$iog=j*9J<am|60Fi=uc;{=AI#BCKou-Nrr(Kb'
    ';IpWrUtuAd)-f<1>2<iA_1@axvtjX0O53mB&k7!2uD+F3{~D`K&$5b^g{5fXSmJl9L-F%u*oEqX5A&~H&T7PPZyCRnRpVz_MbpAk'
    'G{;zbIO+WuMf+B8k345sKTE2=$$37@ZvX{Lo&^;R3$!v#T5TOC{#x;Z-@O*vt3LfSshHw8e<`Z=EUM^Np!*el_i7)D|Im&qmZ^W?'
    '&!TF2!qRN`=U4olsH8uOD%us6s5wM;!1K%_(%&(!M9qGxeld@3e0Zk#W>kH>z2l_6eqL1{gKwW?HNM7*Kl|(Rzp;vbg@tO4Hg{s?'
    '{YgdF%s0{N!uVNI@}Wrwe<`U6Pm+@8R#>X0ZdLxcc+!7!a`!v%?AU&o6@T7ie<dq=npLzbELRh*i1b5G0d;DgJ|}$ctIuK89+cJo'
    '4p#UytLRsd`xU&6^TWEjvNfBaJ%#)%E6?`+m2{v#OD9?smZu38F-ONqm(%3L9@>mvjQf*xKFl)!_@%5ydzMwSC@fIZdu|@xrv8lj'
    ')eql)cqLi?*ZH05cd%;mEURcySfD1_xY0*yLiR%y4qB(FR2i(shiKGas?o=yXIVv;V!BJAznuHCMQ;b_!mlX+_*qu{!}D+drL5#>'
    'H4$wJOVqSpwsoFMMWd~G%@AWg-+lXss@LyE)jmA`zFefk*HH1((jwXv7O6ROAHf}Mf6L1He)n7|)VtSsn$?Fl!T*5jV-xuJ2~zx+'
    '6g_=dMU%oJH9<d=oSSs?;!FPO2wm@*@PnY_?frtH!M_v~&L=^w=uudp<{)0K0PpX;JhuYuO?MlfKM0E7@Z;E!UkM8ESy0iSuslus'
    '*?lbip{KwKfgM!S9S6^#qLBV>Qu9Nb&dY^5evQ<4)}UR{p0G5{!O-mj+{V39*A=xD1zJw+kre1M9rMqx<GV@G+xtHr=F3m1`887g'
    '+3W9$R)wW$!ffMSN$ARGdvdlydf=mfGP%DO6265(+QzT8`)2<VrM{lk5-XY%;&l{^!?PI=J0Gk)ude}~Iuxzg|6e>Kew`G2`14<V'
    'lbZZBP{PlGiVlUiL$P+?>7nyO&l@+<tvZR0XViELYYM)b)c9})zWRvbuWfvtpB59*pRfo`91H!0a2~clp|$Ixy+Hav3f^9-`uf|u'
    'NDBY;#iZnmYid3%CZaoG0h&mIESBt0ojwV+L(2<Kg-L*)n`ge4RQp(%DX(@XmV6BrJ?l=a=uTLGCc=OR#p%3g4{Mg*$EoWV=tI`Z'
    'w;xph(;gN)e=92L?Q5vuX-N_7iRJc0-_q`Io3fA>Z&I|dmv%a-=I46kZ)P<<Tfi@6HOALijVB!nT^o^=4-)e^n78J{|ILq;QvogT'
    '+sAISZ-xYKBh^PD_}Znn_wg*HXir#tCSI;<JDgLp?^CPMIKOzv=OC(YW|e)RBK)HIYwcN9(V(#WOuUTOLodBp3!_hO(H|X8FZAo}'
    '{T#Rc?M~AF(A)o2x!A8SzERN^4T|>cjcT-LP*{GZjfk~sXSu&nt&Z-ruMXBBcTWoS_rI^1#P63Bk6piSZB3;4MO6G`C;e#eT?+C5'
    'P10Rtu1Cdwd*O+Be@9Xur;2_vD1M_*K>K1&|DcHe6{iW$a-zt|<OJ=$M7iJm)*nvYzMF&c!~XXEA(y^?MLm9+`d(7<wHJ>@<5^NO'
    'k|HaNT|CZ36jL`nlN7v!+dLdoINrjW`}Zq~$FIHLOKN@XI@Pc}ONt{Y6%<Q#op*ZN52c@NdY!YyEdU-t1zq<2dZM>SfwbS*=0WRg'
    'W>OpLxi*i{Byuu`!m@eJX;(I{;0jJ(r`{?2EW5s&Q~%K9@!EbI|MR~Bs`Im;Rsdz$ndsW@IFL`;_2OaE-o{Dy;amHDIO8o?iS*hQ'
    '#M!_09_4YqPr>Oe!Z8K0bc6Ro%-g%}vd7<@nKS@S70&p%8tl7K*^t{{puZT^mn_-tx6$WN^@z$`iDjqFc?6&S4sbyzt>M8y(m(RW'
    '22#fl4cM;*HRx-gWIPLML{L5_%m%v|j{7C0U;1vp9!zlL6cPu%U+Q&w3pE+=S_v^>d<_)0XF)*(WfGz>`k0)<!{NFz4TdZ{B|GE%'
    'q{P0P)clac_}WN=iS28oNS-AX4GGKovj|{0Psw4qCdH0{?F5QnTs84OogXJ9edqOQ{$pb1zaWLrT1P!v5|)^WUOcgUW4)8*bQQSh'
    '`s+uvCoe9W_@6Fs>wWsfi?dB9eNj)qv*uBcmc;NbQC(9W!Fa#@*?b7kV8pqmoKeMq#P5GL{0GbTrL4YIjnvoZ&$5c9gvDiA2h=!6'
    'x=6dGlXeC;ky;-l^b1~i|3y&9x1#!5IZ|Iuo<$XXiLu?*Jz04jp5r?YSBR6nC-{yPwa*g*zMIu}IE3I^2jxupk~Ps~;d6s>MvvA6'
    'c+2Rr>92>*4$E{Qm+FJ$IMR~(%d1=leCQc?F{<@7tuFs?srqlIMl>qGB?S#RYjwLH@5?XiL3>8^X-3L-qxxutyqFcYuc>vJ>seOO'
    'sj#?ALjA_hFTED&?4FW!<eX;ulI~BhedrzZDMA0Wq`syrgyyrPqE%sOnFP@3oRy@)*f+q=tOyP-y#ya>TVHTk&9`$@-1m~|Uo({i'
    ';aO7As{r>ZWZ$dqtDIHNT;cXM+w$}<rKtYR(1>S9$wRgNy{yI;&#GtV?~M`N3Jc35nm#AH<EbLOZ@Er15Dr!?;fJoN?`O5%#y|gt'
    'SF!q<Um^YJ9WeS87M6*+Px&qQwuEB!c?}@7jDvY370iSexV6x7)jGr1ZdP6r`5dZfSfH0>&VyH>N`CmoOYd-vj<+iMoVfShocf2N'
    'q!$m$X8cmEOXq*c${hcVQ#30qE0c5s?8gcUQAO+z^K4OZCsgz8-^7JHLu$O88wb9Z6MaplGxM{YqEmtHR9H=z`LKsOm5O<9gR>Hf'
    'Q(B0UdU2f}z^Avc*P{BGPUnfwqKZZZnM-8j=jl|Du6yH^z6i6gak~BTs|)=AAIS9Y1vS4W)A=9j{Qes#h&BbeoS40z*VO6_$a~x-'
    '%+m}$l2_OH!F;IFzZVsMO{IHA`E#hEOJR{##ElN66br^R`)xrxRlO*t*x>Pk1;tUJ%Zo9|7f-1{&ti%eg#~1ysMhr{V22eYSDGnw'
    'y-%Ior1hy-{JT*-9_stetiJZ7$o=WvFPaqgM1`_PA+~<6(p8BJx08SeA5hKq7WBjmcE9zh?DM6h))xgu>(7#k7KMdnnx8a|ZBxst'
    '{yd;emXF4~1=U~f`(b;_Jf@_5?bbWx>0>Gy6c&}KS1C5*k59%Q|8HancA7_4>&4YC*xw$6hI}upFPgOTA0{RKM^@3Hu&7K#@X0H}'
    '_eT}C<xjudMmt@7=lBrB^Zlg8g9*&v3+js{nsYwAsG>z-Ihne%Jxa6J>#pm+u1C)}c<B>9O&tGzPM>_mmy&|7l6uxmf@o4qmlVL~'
    'ROe;-&!6a5@5d>L(<0a3pH<(F3LcH0|6Wq~MN-dSeWORg{_4Tx_33axHI$K~Y9Q;R{yug}e?O`ZZS$`_r|65Q@Yy>RMwf!!rQqF)'
    'eSc0Nq8Hp+m1sXaPtExj66f(1pguZ_F9r2QJ83@cQDF2atcyUk?$}R>hl9#)z1yf*{WNxE@~6k4??y$bK7}xT5!JI6Ek>Whk}^qX'
    '+j`^hkVrgtz6-9CO`)Vd_TPR#sgEY`OG(N2BB^KZz8GB!OUlH_=(8-_{f$a@&He5H0tfGJ%B!oB{BZqHuKQY2UwfRN<LT`$x)hd{'
    'IdzHidG2zvRp$>>qqS3@^IRXM47~!?d7j7jf?8iRDaNxGUyLS&HGEK;yFcg9MyjXOH6mLkv@@v~KkyP#i7!;EeR202&%XL%^e8MY'
    'Q&-D=oxfjG9iX{NC*gKVbmkA^JmBvq#Xm(VniLk7sXueWU3z{f`7<@=Chh8QB=v75ng2zk`j4%tXi`|hj_!Z#YwG^pqvI89Iu{<B'
    '-jvjbukmjO^<zsadK8wGX};fhu=(-T^5%I52TnakB+kDc)aXA1ilRedL756_c+KU>N<!C$V<JO8f_m+fZ#pOXDNfO#u&7Mc2i|kD'
    '4lShDsCn$&Be@QoLet>ie?$%WDNxa#u$)ZM)#9P14+eScB=x3u<inFMk5`vvW}||iQWF&Y2@A?(-KAVAw1bh}fp=6eD77;x4EE_O'
    'P(7GE@Xe^cy8X>(QN1@PrUzwmr)8_EAId10<qFD#-qw-Siz{f;Nv)qEMUj*ViqUtp?0sAI>T5?d+GNCedh=!Y(BS^pqxxe1z&D_p'
    '5tTU<c32(9L@axf-yK%XhKU_XeVhpOx08~eBE^xEITY3r`k>x#UNdq=gJV|?f1HQ>*OQ|0Q=}x4S{@+cZ}V&1FDe|{pKa5EQ4g;k'
    'jIqY`?_X5JKZR-qRF;>iKe^AS?0An#f8zZ-rKiuaik2}Jr@!F#J3mk2c`2*;)n)Z;A;oNwmA!lDNwuHe?^l&&+HV)PAs;F!m_DWA'
    '{Pn2jPoe4&l{pop`{D!r$5&6b?g#zULZztQ&xx4y_oL$eQ>aEnW!0IJI-JXs2JWI4YYIQc{jq+L?-o^C{~w_Gu~h{TmFWufr>Esb'
    'sdr*s4>Z34+bs<7$SO<-fWM!W{MfRJW`%`lGB+Fls>_N_PruvGZ3G_Gbzl`ouKj<&>c`eqv@0w|lN7^lCAjwh5wP#xY+HsWW%2TH'
    'Vfn1qPqB)Ag#~F&bHMTVScN9pci^8c!FfdW>e`q2sjK*<q}ErRrDtn2XY?zE`xUyU;R6vU==}!LEX86RQN>=WzaJI<6sl-g*u`O5'
    '@b)p8Y{8#go}n82xc|vpe?QfC{QacXPmzj#g@tHN%X7Kk`KHGv>CUI$%c+!BA-%dnDHW1_{XezyaWpC{K9idsKVHv#@c;UsQDNA|'
    '>PJ-XE%kX<^&Habe>tmvdQ_YltfEr^?o{+$(tLRTX!;x5c9qW0zY@Dh$NvYY_@_`srvki0lb9crXhPqkVnvxY#v`kbVe~Ix75Yl>'
    ')vVSRS;4ccqE+Dunl^*cJFjn1{ciyb=<SzIsMcE;xhpS0^<aR`_oDj8k>>vbRe!dffTL4k0h+9znU6yx*7y^5C3EpQlKNL=n$M8p'
    'kAqiU3u=53)U&Nr9Bm3q&lJ5SjdMBe_5Lf1dOvl2D5!sxOz{jT`Q!z^mK1!ARC~5No1;l#;hA&vZT=lzs#e?M5?SOpeK`Zw`ea%C'
    'MpX24;tNNY0@)m*>7AtaJ-@6<eYS0>6O11vMeS2Q@vkJs&py?2G%3*gKKg!!KS*_g-JmF{68idCRG%xXe<dn<dQnA}f=p60XycqC'
    'g1YS79ArJ}kbf2xWcFu&9Th$s`6AJ)unbL$r{MFTOK;}b@(-7%IQ}Fn%FL1bI;;NdvXbakkb4#6B@@TzU)*K2OLxHb!#f~<&Z_&3'
    'tZ+TeDw-9Rq-l)>57HS`dX9YoX7KivpJXMeu~%Pb^>mV)M6<$@G`Xz~4~!)guPd{}XIw|WcD;J!<!m4RdwS`_w;NWEy>+B#C%#E^'
    'D=bS>x(X_9m(SY+pFlMp4rcxpq~O`%q!R54%hF`k>u`)wx5k5=P}Xep<&T2GN1><RIPUlBoc>`b%YPg9E77d598Crr^>I9@kgvIX'
    '&>iOwa+;q)Fg|els1APw6s*r%-~4Z&qE#_%hPe&KK9;^v8?ftcb@Sz)T~PcMwBK(9^$+6{|0AgAR9K8Av%P2-kHeK}ZImp6$1IW8'
    'lafb~f4_s&oX?VqMg@EOM_<189j;W9_5X!kP3U)fCiU_E<G+~{xA^$``9Ae=V(F8lzLxk0^!f3F{}ZVdeF}Cpp<04mf8Q@E+;c8<'
    ';jMMneIe}r`%9E8ujGTUdo8K2wQ0jY#AyCUQqiTb7){ZCkXMSG^u#WHktHGz-l^V4{C<C#5|6hqKtf&(>T6Yh@DDk1{}EKQC@e%%'
    '{~r^sgv|TRuWye+hTgl*x84`$i72U$HSn(`^|h)$=ua=HXi!*)rajW{;IaG7Pp@1%@Xxg4sp?tUhdD&wj|x83RlgL~7qgV$AEvVW'
    'M^w?DuoO*8D8q9yE#lSofsN(Vq^P7`+eyIq=r;AupuQ+58b1pv+7tHT&ZBj+e)+JbSOM*(540w^BdHe$zwn32v@c&%^NXn9Sya)X'
    'uoO)_s5+m^(}~7((P2P&97(-+EP;{=(YN$!QeTu5?P--BEec`%zF5C~mlP+JzO&?H=m7+9MD?-F>HATAtgU`6D)}O+_AIJsQiz)r'
    '13Y^1fNJVl#e4q)dY~x^)i<GvF2(XU4_NE0DJJjsL|L?O7O4bM{s+7#wME6;saA(Yg#*1wXZ2&#Z;|>wQqiSY?ovpbUVwBwsdmq='
    'AmE}xm4Pby6qcsRg59gh?mb$2!`d(SFxxkfn!#uN3sFf1s%TVrnx^Giog*42?U{THmlxd@sRUB*eG2w4&1ExwO;g-A=uq3u**5KC'
    'orW?Os&7O^5ta3Q=vV53feuc^fbIoK+}kKrIjCktWln|OZFfZN0Ts33LzxGTUytg0%agtl75+C=98p;xseb?SRG>NZ8x?mmM0V4`'
    'lb+IFMT&olltfa?0$*5sfZ~0N4q1WbpNPiOe1d;z)V>zg$H9OvaB9DX3jQ0a6;N56=9=9vRdVRj(c9NfK1KZk1{SLfR=q`5Dl05^'
    '(OPhXj{Q_Tc9|0P-@dHAw?O?HQH}qGsz+30RZ2AITuM7F_Rnv?Og<F2P$i%m5fvM%b}-c0?!TT4Eu)kCG^GdcRKJ8$f0hzNO8)qv'
    'KfJ#v9y}#0Qu2iE-sG&)|HV|&zls$76shP_SdQjmaJT&6Q%vc*ImZn32}LS{RJ189MUz#d^O=-bJnbG&1%OnfGDt<2!ZI{f@p<Bl'
    'M~2_J6tLV;pSt}~ZGS1KqDx^Rn#<k_z3ln^J~glD*#cIsS)?*ZMVG=dG-b6NJt2p33Ker+_y;nm4yqJV(WC%3DPZiY(!r!?m!Gcp'
    'E_v`wRj9rP)rcO2C22}y@q!9D#K>*SU4LiiI;T=VMTf#-G;K8E9+7wts?+FKg7B$3r$BulsOV3?_9vE=Js*xKma(gzReX$IpfW&3'
    'i^38#SL^J`@7|%X?g|IRoo5NZ6j0HjK$jB}_<@hEfc7MGCayy&hg5VZEI`wKaNGOV;dNecHv~5z8nvcUNJWRj0yJd|kO!@a3vhes'
    '))-oADuq-uC@ep-?`G>$x3Au}_Fbx5!$*P202TcSi_bJM2j`d<>m1lq`*fEq&o75m^e3oWafI$@beu@9I%4?S)aae+l*#}F(VejD'
    '%*DJt@_Tbu{`g4LWYncM1ypn=$jXg$KfCXp65SU4iLuFQv%J0(NYR<F#7tdHI3GN9&=ndy2YcjPk$OF;XiZpNra%MV)J{cy(0TV`'
    '`_!ER^?Fdzny|1;z0K|m@5w~!9sCeun?qmnJOx$sCM+#;UCFX?Tje~Ob=Pmd5BWjoXp#CRQqh~Ryi6NH4-dQvu>wD@o?!L~BZcbq'
    'sG>PxiJ7|lH93U^554EYPOQMOYWqtf72OGQ&{c#FPop>-R1;R;^TI|svpJxmIbn&Jx=w~YOWk((j@B7GS)?r{EbW&83Zpq;fthPI'
    'p68wcA0=Un&Lm*%eL19}JHa%qMPdOS=en&+qqjcb`FrpC5q(yM_Sb=08K9y=VS$-zcJcN$zSrn|%#8{ctX`xtNJWRj0yEd@(;qwJ'
    '{xSu)H$UsH#sZZADmoNmZyj@wz&Q*bU2R<<WY!1AMJj_-G$<@Da|wH>{qg;<@ApHiN$+(`rI3mSg~esAF?U5Q(EaYWdcRDJg;1n2'
    'NJW2Q$7R~EqH`mT*my9PeyIPp7F7zWXi$iMn%y#WC~&@pi0Rt0p_T<I15`99RQ0i-t`{S9rzotew2K=Q#Jv@$3{Vseiskte;yyax'
    'YxH4W)2>8^=O5+^)V~20{Rs<e=^zv6P~Nv+r9}UltCmv=rD#qpE5z(gyd>-pKOrFdRo;fNKJTFys(*tjx)Y}L7WBR+RNmW2R@ChE'
    '6H2vgq!du^-3hUfOx4}uj~&NJZXG<Dr?D+ADO4G#D5COOOv`{f=yO#QeW!DaxlU4+Uka%iNqI4*jCD$$HFtu1gqS_fgOojyLW(0P'
    '_CdG%f$&h0G<QV05VTo>A_bI0Q1)rK%h^Y=Vx&EBR2NkrWuxbyS^-tJfe$jSRP8%bU=Jm<Uwc*i)j5?xYPLuU>_VlB>ZJWtQLHh&'
    'jDp%~Wuxbi>XDQg6o~qXj`t}`9CJm{8jQM6rGOd{lz9{Yo@9S`mbVu~ubj0hmUT*{fPx5Wx#fp0C{D3sqq&KWj8KPE3aIE%jIN-D'
    '#rauX*h!*$+*R+4RHk1Jsc2B>f56iOIGxHTEXmY!$pD&%))uJOgNpvdAiwnjdN%O=WeVVu6WDkas0>iio*1eo1`@CH2qR%eyzaa>'
    '>y%0%75xeOG=21|&gEYBb=;whxCU~ON+A{P2{1j;CXIEr(eWBa-_80fPzj(ox)XYf;x?_s(rQ@l?FsDqJAC(3^-(r@4ytHRfY}oh'
    'nT`#ky8Sisa?GXwPOwCJ3aV&OfEg5n$=zAN!!zRC+OH2!y(&_#ClwtEFiFw)nhSBTXWNw!rw7)e+W%5eMUMhaQ>X~{p@{u{`$Jm0'
    '52Q)G%;$iL76sai2k4LOmF9<oYR&G$NV}{FwWd-~MUw)pXH=TMJc*o!P03Y{K6;<kIgo-X+7vq>lR(ZL^Vt43*GgJ1^_M~_+7!54'
    'sP9QChq}z!VXr$YOt0-Pg;caD$Zd+|%I*-Bq|$GP`sqnGt9c-WltiC`OjA(bhSL4+2i%rVVqQ3_9VZ7=bSW$&Q}x20G<EQ3iP)cf'
    'r+)vXCsIH~kAh53tg+R?{gW>!$g5{mznsz&DWsxDLH22A;CaxLZGL|SO|@E6Nu;7hF|E-9miVpqu%=L8`fq8L1yiIlNJWognx4?b'
    'B!?6V1vSQgpCWF(<U|UmXi!X(6TBwUkyCGeNOgg42wp5s|Atd^C#I?i?8|!*WgTK&t^gjK7THszGDt;tVlEfxuFns}v*Vhy(scu@'
    'YL^^P(VSp2Cq%@mj^w_ELT}1@eur4@Y>~<!wW2w}-r~W#5&M2g=_lJRIs!p;_oje~=7dFLjxoBo?%umd!`7Y4zKMO*l1d>J-3c~#'
    '0()~0eH`kN_-dQdFNmZBMGC5DPp}CJ(r?wyse?N)Rr>AW$~@}Mn*u7j6Q;Pwv^wVV?x(*;_IvQ0bxfszispo<o)~@k2&03IgwVWZ'
    'y;&z>ol+^FqB|kxPS`L+r*IES=A|cOG1T6dLMr+b7Lf^EH$b0@gNO2otEFB)BUVLb3aRK%SVE@NE_)60ZB~*91N);qAkI~5Duq-u'
    'C@dn=>>+UBcmLK)wEJ=TB!XUwA_0}IjEKq<MPn~x>4z+5+Qax-w;s0wl>sU`6c&=Hi)Hz!z0+licS2*QQPt=<prS!xIhi(cdF|SW'
    'gP@q#Q8otETW=1kXi!)!EZ~^y<ofmIQKr5>&OTSwPo$7~?@+8AlnH<4J{(%+2l|?7z*23aDWE8Vf@wilKEY`9`z5s-JZvB=w#swP'
    'AvGf@@^gyLwdjbQcYlXH_!g%OP8>Ozo){|X@K}mIWfyG9vU#KhDg%^6P-_Y<=DqjbPDB(&V9$Hz=7OY1y`Izxq|EL!ht?JPefk*6'
    'Rp|n{^;Je@3aQ>ADZS6a!fxs3{r;3%XkOnIer5UPfa(#H^^4lAmrj$g8`6~)g1QA(Yg-Pf5lMNf70f;P;a*U%{>wY2^e%=}yI%?_'
    'h^Usm%kL@5aws2DiMSJj!5R{aR0gT&PgqQ*>TmH^#BnI}+hdFtqf|V44yfo)SjZpr<`#LA!+Qr^4#kkhb?;3f75xc|$+VgypNL5J'
    'KKdN6%Qi{xRW)Qe2UWBvEGQF5|9XT9uS&lr`W2K(k%B6k6!vNCSsqVC6lVKfsaD+@+zM0%sOV8xP$uZD3eLr|WVwsZ(ymtZHkCnY'
    'M32IfGACcrigJiZ-?e^yQtMTfnL{d?6c&^TYYaI(PFbpp>~?v(PnAcJ0xEhG7Ly4w*MrUhvWA^4CkV<-${`gk3a}CiZdW_b{d+Gj'
    'pq-K4Dl8<4RJ158B@?}K&HRT`r5~)mM|sRuH54hNqDO&NLxHl~OuCK9)I~LTo9Uupd!b4}6-^4<&mP`6b(k?f$J(NUXRA`b98l4s'
    'uz3PBsQJtL?K#z<!}1n4TGA`nC5KdWC@drsgo)3ENhDYKY@0Q_JPA~ED5!h&tO~o+l4?tu?|q_jLb#Sx1}TURg=J*wAw~1-P*_on'
    'FR9sBR*Cc!QqiFxngZ>bGEet4WcHg}P)xl8V>#(5sG>(fswl9@9(aAPDB8FxXa%^U>~laxi-ML(zqaNnA-L;b(<hiW;5`haNWGp^'
    'v?x3!bB)#4F?n{*D}{&q1Smz3LMmDmmXfLT*r$Wt8?-$8&c4#032Qn;4you+Oq%wy%W>eI9aC+1MIdv{uuK9K9SRT1B-$0U_m5LD'
    'Ox}A@I{Wn4BK3Mw(V?)QOdQnP`594F2gwei=oLVg11dTc?0Q1%GoOmmnXc}Efs-pJGldjJhk{{_;kN6&=hvGMY*!F<ldMMU98l4q'
    'u%JxjD}?9pG;g`5UO%gQpj1C%4ytHSSW@QdvPVhXKO-)NZBW?cm2xO@NJWQ&t%$;IRG#fZhpu^r<=U^=^{Y^2po$iS^eHyK9FJzw'
    '!ZFFsB$bJ)hVA8$iWY@<6$OZ)x*t5CGD3#KTwrHak;)(yEefGI^c&qA?wxe~VAJ0`P^QX|B9%cZdK4CxNn<rWN{Z1$0Cw-_AgrqN'
    '98}Syu(V8Eu*0|c<k0RC+P;-XfqFftXi}_h?})e`=jSPeY(IJcac@=ZmqCi6NwHK>5QtZ$AI_=17oEZsRqLhFQ%FUZ!oo7G6&}wu'
    'WGv$vi%6pa?Q=jyk7BV(KPIcpY>nr=_G8(Bny@-cX}=Uy(WKD*jP)|b59%`8*b$5}#Rhv6sSHx@JPK+amZ?9s4<9%L&`P^R`qZzk'
    'h7aVBqDTs+{w=Trn{U%H^+)4<<&r@QO8cddnvs;1QIOt|@T}73Y+h+WO{$I}i4;dtCMoph3#XuTRTQvzY&S`&cPa;zL{Li)p0-QQ'
    '<p8Sjmhpp!)QU<0wF0Q#bw%~&-s^2-lHPmqdY!lPsz{}fnk|xoY0J#xx8)v`Slf=(E=?RYb|nW?kD$n&Fcj48;kei6`qdAY@q_BY'
    'H;2@Sq^!+jDy`%Ea!WY4lC5-2mYhfd1rgM;Qwqm9iJ}i(RIoQcdqyl$8Kj~=F}exA3CDIs6Mh4H?_*-D`q>mv(ViG8CnoF*?dib-'
    'aT`fo4Zc~RqB}86ORPV3-hHK?T;p+utXYm<4you)43iUM46lv7zw|a0=6g7p?tZ2FQb0v_VwgLjf9icXrj3QzIhuWNO+`$i6s-xH'
    '%z>SSAG{wn?mH%PAyzed3Mr1(1X%HeZfosq%QgmYZ)?ON73-1$Dq0gjy75h7f0Cbv-LI{FCC{#kGtD6ttqHJ~4jBr`HoyCOloB3p'
    '-2#ug@#c_<-h`!On)3EX+`~mmFI#gDCa?soB9%cZdJ~qCIVXra=x{%9YVQ3}sqLt_BK3Mw(VVc9OthRn@0UM3OG(>92UZgqrP5PS'
    'MSB9h!K2%xqEq-{w=L4w)Wlw;V>zUvJ)z_T0UDjV25eyY^1@jhQrQMMq@q88=1*9fwvjq)f2t5Y(R2;bbyeG63aV&QSWqT$Te}l+'
    '9vTO>P&e04X056WR1#eZa+d<gHn)CwnwnS7D7QVduSmU~RJ19G{1H+7@=J%#0g-Je+1EeHQA$A-eG2mIZESEZp%^ys$Ro(8S4CzH'
    'sAyAoQl{wbcOBH9g6e&bB-NYmC{j73qD?WaB-8dYzt|q$J^pl!>K8pL+$9H9^eLtlQ7FHThlzVukKQku9qUGw11h={mXxWts^)2N'
    'hhs{4Iw$~;(tbIlqDwJNP%QmV{qY?Sk<Md>kRkJ|yKfGvXj4q<g11pBhfq^bb7r57sQ2HRx{|{wdK7FP1==O)T$8!3t|^NvYxP2v'
    'L25;d!lE*5()>CXQuMW-=VdMx>qRPsRJ17Ao;~DRQgEn9?>i?Lfn^k_6jITlV5_9izspe_%%pBPAuc+ag=-e73{=sh;O$ZD5T1LL'
    '4hUnfyQN~jRC)@j=uwDD3b9|j_UQhk0#;4oZ}=TYul+9tRrDytldbex<u!kYUhH*MXEw{LtyPtRD!LRNmMLxVF*2k>-vgIrQlC(Z'
    'Duq;ZDa2ih);+%9UjFH)_E6^ri=~!S3aIE&h;@##3P4*g{ZO8CwGfbuRituAMVG>wcCiC^CB-4peqQ62r=lv0HiMK3he*m@ie2${'
    's7*)xV7PPAo3zSFPazduinRp=%Y$xgxV&^d&t~~aIi#XXVPTo1(r<@l(qU1NxT3(R_o;gJNI?}{3J=R<7(N^Owv9j+Romm<I#m}z'
    '5~%kkMYpWXAziOZrwZDu(M@m1RYH*ii6SHw6RcaAtBI47L>;_=V0l+up~^rtBPy~>ufTqtx<`ex?LfJym*1MUoP&xZD)!)Ccu<xJ'
    'eR{Ru5}SHi5<P{KL{gqbf#j`jaHw_e*Xuli%t}emLA3&^?(@Nr=c(FHZ&JO{9=gYE)jMJasNNzdP+13k@Hyr{2+4!gZ4azLmPx33'
    'L`A02l@uQHXJKAB2R8PtRr_cPsS!!BhbH;t!*&WzTFWH`seCL&kpc=LD6gVeQ_gQwsJq`zxI&mv21N>~Xi!*Kre3F{``!1jrnv7g'
    'GNM_5_DQIsP2q8wV~j&+66$)|UJ`eJ+WJx`MTcTo<EVc3<2Y}xBP%LT-sVxAQ%R(vL19^$lg1tnHtrV{0bWJvy-T(D=AeoWg{NiC'
    ')%IKN!S?Gr)7jE^ux2CXpo$KKMP^Q(u+HzjB>QE4SZq(FI;W5t(V_r<_E1%4x{eRq-wwF4F)g*EazI6c0?4`o`i}R*`z|+Wo`GOx'
    'Ne#+MN+A^u3X9BS?z<)Deo4XN9lzaejCL96DWsx90siK(r%oLrGf_6Eh1N(_re6xF=un_J6b(;N!Mt-#?bVmCAf-Tx-h`!P+7qIe'
    'P#pBRMb~=C=9m?1l|(A~6X?NKvgDi>GEv4(K3%JxrE)+;djefcuot6y0{X3qepMx@mM1A^HU(9*CoC*e+9GQxU_PK~bM07Xm{J84'
    'IjEvNVPTmpS6S>j*t7k5W<;c#VVOb-qCH_*nNx4Q{cNPqb<OYX`t5O%N+A{f3Cqfqd9`o!)9$wm{X~^wol_~GqCG)8g@*u~gRK_t'
    'soB=9C|rfkIjEvPK|Z~F?8uP&-H-KqadUR4o`8CrN+A{f3Cqe9GEH$z3_<(O+({zFUfM5(R5U5(e(=~`)^J)>=!qe^s%TWWK@O_u'
    'Qdn3fFOa^oW$z!SaCZ+#$5j`q98%Gym|m0F!A1Dt?#ox7hv;5ORX~=5D%up&Yzo!hkLqVhZ^wK5z{1r_RHUGaHihM7G7LmKj>hJB'
    'g#~vQ)rp+~3ZqM5ahX1s+?LCIxr?4>?4$PvOQ(_)DWsxHVR4y)jUV~x`jvIKGVSZ-BZpM9DJ(8i*vipuR^|Pg(p9ya+&XfNh|M7t'
    'T?)4N?a_S#nB3o}7EhczE^a%7Me6mWqD$dHnWpU4u_MRMD3^YtgHS!tr;v&+g{~vL_`<YomwsDAW^Nz-sY<U*`y5cwqY$5Dj^OW4'
    '_Y$4^XuEN$Q$>*FfQlA{C1onC>8kFYl;60vo|vtIf^$ekhr*&Vm$f232uh!$?LIfwe^vKX4yx!-EPM7~VRI<t;H7t~vfW+TSjsO0'
    '6h()^qB425Jy4zwR+9d~t|!(u4vW<5NkxakqB2c*+bFj~O}f1w`n{KgNJ>$pkctk4MP-VvD)*A`e)kh;dyoY68jhVrDmoO)-aJU}'
    'OY7L=#%=f@%MJ@$RgvY8dhbvSi^>#1*_o!d=hPC~712vn`_+<40Ywp%3JMk7EazEnOjk^>Zj(NrrAWP=)QqG^mlRpB*kL;2V!&y>'
    '-)J(C%1TNB#SxV03A6mpr<BO_s`M@(s~n0PQW8n63D#)n?kpaB^m#=Ew^>n{Nu*XF)jip2B6zId{l#~6bR%8*0<P^Z1=Va3l`0BZ'
    'usb>KFTN@#Tv1rg)v9!>98x`!BHjK#f9!LtX_ww%Py86I5)P73jfhG$g?*s?_;m0nx{Ter1Hx2S-xO33QQ5PHg2D%dnS#jPLdD!G'
    'Gbx8uG%1FS>gdbs((j)V^=hp5<Ly!4s0i8|QqiTbv`q2hq+`L$LZqu5ZuL=R=_#b5OJOJ1YP>rL4uur5OK5wODXlKQDWsxJG1x3x'
    'mFoBQ?COp72y<IfCHzt-MT=sXMS%kLGJh&bUsqug6JR|f=8%dOg=J+f!rkP1u>B_Nr+GcidV$IS#nGU!s7%2%=m=^w;OV8%#QHx~'
    '?UF(&8Wdn+g1OCOR%T~v;_=%9EsM5DWsr*g1bCOnL~o0_CuK^)@5R7vm7Yie72OGo%3PMxe17j0nOjeYo@sTFN&ywk3G6ETc&u}!'
    '->{*IyIZ?s4PD6r6}<^d%Ul9J;Nx~R?P*iG`%#}oDTPw>B`hg(Sp~&9&Key}@5G*LFjuSfltU`|5*C%IqL00N;y!>($ZPysTQIqr'
    '+j2-nZvyq_o-}sz1{}sB!jL_}ZJLo@s4`GRbHb7`7xwLr%l+<Wj~w<pY~m=FEt^3~qB}uekh#Y6R-F5n-gQlyXOY_cQaD9#g2?rG'
    'pB|Eas7RWZKnK0?mN$_?DtZ%Sf?{n@hblj@{c%T{0`@9#P9YV|3377+y$$jHnO8MBIZX2fL26B<kc#fa>=Sfv6Q78EFVgh{+m&NW'
    'zrO0k$srZ(3Cqb`mObW%-@IQ`t<&7k9XMu*^b}OlqnIWs`r5VT2U5yMwu#QIW|!xXiXO#ukD~4Ry<1Ryo43|9rHijXrGSbS#az=8'
    '3%b8$9E#EBPP4b|PpV%&hg5VZ*lJiTSn+sFEg32jeShp#ik<^%MT5dZGFO{Bdh!00vd4H2HUI#rK))PP(V}3}6E>OrT#>ZqzE0^r'
    'IiQwQ3aIE%u*->EJ<$(xLdLFc=2b!VIh>+D!RAja-ZKXdkMgC^e*jf9D5U$|=g%1j8+|A%-bwZzD_5v8P(_ErdgYtF&hPr=`&Gqq'
    'P0jX6iiIizRdguCA3gL#lNX&IRu$~A*(Uy^YG+eWMT<h%Ru#{43dLx8#<}bO1zTQv4you-h_!e?C$U9!|LWm$2lmXB0d-R4kcu9~'
    '@|sNP>pXQ}bc^!ar<oph)#Q<YO82mc3iRjho%(V6Ta3Vdr?OnRQHv^tRCFnpYl=C%Wpv=lO&7m;b$d6inwJz*(WlU>FLV_Z<3SN^'
    'pF4cJ>ym;hPs%|RtqP0ET-3J=zt`!*XS4VSf>oVUIiTJf6<|r3F!W{%$G9Z@crx2ABNLy@SEOD~iXtiVDO90b>y#W`eN{<#%?lx~'
    'Q7r^1q-G>Vc204(nNtTpz0KBlRMOIous~&i;t0x23Sbl8hv8s)NDXhrUUL%E7`Gf!5=mJh1!CiK#f$!Jew}J_RS9hlrxiGLvnVF@'
    'M{)ugJ=}m9d#$JxQoTh|DkoH)H#&zKs4Q=8&`uGVwWd-?^+?Km*V%pO%Kf8<*|N(s(t&8zt8Wge5lNYzP)fdF#^HIYdwZ!pPpSAQ'
    'QW>Nmk}^Fpt`^=yyn%k}UhYS;l3kKOMSo(LKS9g`y-4Dg)I`}&zE;Iq83#G2qCqjtpwLf+=O*7SyN*nC1#_9RDWIZ1;USsRN|{Zh'
    '-|v3lkxAyYD}Yw0Qcy*QV%U?1YVh;OsbRIVewNW9sJiqVQqiICq|DVlR<<3qpFIojRu7n~&Z!hq(W3xs9@R@0o(IhjdlvR-N7!Lh'
    '{A>!U5lsqkNpYn)-*11QU%X2S9=Z?K)i;GyG%0{or|Z$vc8S>iNyT&DeJ#4DZd56xqDi5U7Ro>$J8-}KEn7=_gt_P^EcKUyD!LTl'
    '?W2gEE<5iFUhII{Wn_8>o>lLhf-3qHmX|s7Gws^ULutBx>fP;zy|%v;P|>A8l@uN9EyMJ};qp6?+)hWmDUVuHDX5}PfmTOBLU2w>'
    'hsDsZCn_jVDWIZDf#y<J1>ZU2g;b}zOVRBE7N`tR(WF2V6szw!ZsTxD$>k??Y^zF>Qb<LUg3P2)iQp6tW$DzWi}W_Fm7qu^kb>w@'
    'kO>M#A2`bEASm=cwTqqgT&t?|98}Syu+U8Fp7M_C`&KvW$=Pl?Q*X8XrI3my1+hK*PA{Q7RA#D<M;GaKq>bvyP9YUN3euljDUE)d'
    '=&@+EovqaEfw<m1Qb<LM!b39|Jlc63%%m}JPu(NvHEuqKRP-n&*r;XLS?ITGN+zSso$~?}!<|GbS`?O<*<tM@#`|6m`_^~3bHiR8'
    '**T=5M=?!NOf-l5)IOy*WZEiy&MBoRQb<LU!a_5L3OipfeDElC9T|(RsG7i%Ln@jS?9HRN!zK>KA2QH>!e3D1mUB>HbSW4VRoh$J'
    '{jhmUT-tBGL{RgmazI6o!csF;rZ>CR_WOAe1Lkbo#~}R%YE7k}iY5iyvj<!C(fQe<jp<T;bn;T%BK3Mw(WKzzb`m^j%(1>jSMo5%'
    'tR<BKDtZ)Rq0N?auuhG>tJ(bC=DreA)luY-iXMf<X4;%ycW4ir-@ImzGL2S(A_Y{mD1`Jvh5GfQi-r4oKd_2)JEovhl~JUSiWY@s'
    'W)AUYn^SOzUbyxtWpher>4_9l(W0=-Oz5~XgYWk~yb{w<#YZ_5IiR9LvAir(|1RsDKQ#JI{fJ&eMsw64^BhzZ9f}2RCT)x2Bq&CA'
    'ivmF^+c}9;v?wexb4+U>JvPtJepS$_w5Wk@DV(A|VR@PQL$5~?-6H**Qxn9xOr?;D{>1XS%<*_dftR~j;Y+7FrBX<}Hz?5aGWD87'
    'PKpms5?_&V*suzda!64mWqJazI|ewsc*sE469~O}M{`KcNXni(Y@*&ej&z%Yce%@Ay;;>cl|qUmDYnOX*lEGH4u$2Y&pmu^Vy}B|'
    '3Mh%7?7?H%hGqp4_pgYv3H!X?AtV*Vn1gBsR2G>z^d>uw#eP;oHh7y!uxj^9AvIehWr_lM?tzx~i;72g7ejvnSiNL=3aTDasieR*'
    '$1Wl}v`-Bicy4>-qN+;IAvGc?@APnkw)IoM-`tZD_e)~ks8UctL}l8q0U@4GDw_}6+Et`fCshim=u%i}CQv`*P%vISZp>URdJd!L'
    'Pgq<gjJDUx>Dt?Nyn$-;J}QSN2URpEtaMDj4Yb?T;fUhJeJ5}?DN;$KqCsJKnZW3InQ~I-2)LhnW!5a;98%Gt7)w8lZhm|kk|Q{!'
    '?^F@us`Ja?6zvI1%d}}mmTTYX{%uNl&z+Km`Wb8U%Rt4^pRlw{|I5yx)9yF1dj_Kn-75=K3aaQ&fOmN;@X1q$QND9w-#t6fsfs94'
    'NJW1FTv3c>`35H=9k!9)0q3gmusNinJz;5?VAFHfd6KXGxt{B6eFm$<Uk<A1Pgq(e>Q~|+jE8cUJ#t>vA8-Z6<$#JN1#XjK&)w_C'
    'o9`~l?6As5&mk2(3Jc2wZsNl3UZ%6a*m1(ru~qXx3aRK(SX!nTN?vb%drSe49#CGp)aR^0zB#0#M`3Z9D(7}#(&3y!J<oPnX2a4G'
    'DWIZBL54{5VoRrV1O3R1{xa1%V|8Mukdo+8kb4xkCxD&Yq_K1TtT?8WUka$`P*_%`?rWo;%V+h&?QjDn#%0i^fQkkMxj|v6vZlXp'
    'L9IXWvrEsY>Ke@<6%7h9IYDiWa|Vwznl908W;501mqIG~6Vtmqa1WEe-~0rwd2gM7q1IFisc2A4)f3o3=sa+q=_;aNUa;z%N+A^u'
    '3X94F)Q{y)^Y>L0HkiQ2?t!dUPaY|xqCsI%nS`_+<WT6>-uEC&Zd{qBDWIZ1Va+%=*ejUVVV05JjQ4FykVf?m<e-WUg+*ne`FZ}9'
    'dr!*#V6XKfUNw3kh17};1>0qcR4?2|CE-}T?@=BT>n@c;DcTbjlZlFs&e@9#rFnO+3iKL}lLIQ+6Bd%Gpq4jy%zIL@I`6K5ChlVt'
    's@J26{)DAuf-V?8fbP|VwU)YwUj(Bn{c=b}e?nAEP#14RJQO7f>_WfA0#fd54you*SWKo$^nFu0oKwX9a<B4`>IpFiRJ13=1ciR5'
    '><?2XML}~1J_FQzh#XSUp0Joq?0v77TN@p8bM#mF(F8`-Pvnq_{)EM3lEt;`zJSB6cbB?|wQSdQDuq<ECoCruM|&{G2%7AtDg9!w'
    'kFqBcNO6nygvDf%ey@D?X7x+=lB70y)eyHFP|=}KNoO(jzN8LA+z_t`mfcd53RDKD=ulWpCbHg-+@ZKH$n0(HJky~cvo2F9q@qEw'
    'ydG1xG&=5mW9O6EsL@{i9XX)h`xE3TnW$Icc5a$l()R2yV5y#|98eTNnVPWAL?=0+KgqZP{b2B@y)T8-jHJw;fJWyBM9@$5zcEP*'
    'N};Ea;z-K$#2R~!-r<<CMo~ALEcdxUWq^_h%Ipc1-FzOk2OXREnrhPO)|-TC1yt4tKtST|?BSe(PHqe0GFz!gC6MYZl2Sz><`3CL'
    '>9F~Y-H6jn0oLx90;)$)<e=P~U5D3RxH@rki{z?C&mlD;DduKjj0;aQ5ebB@0*X0HP~?Du2+I5k=^k1j_w|@6C~)oavBqQ6#Ww|2'
    'bSNw#llAAObEMzudoEbh?;q=&${`gk3d_kv%V(yeLm|W0)M$E3E{UGPDH;?Ol1Zz3Namr`1v-F_o9YdSx!&bdP(_ErQZh-ue2+($'
    'dhqx~prS)zwU%b%x%+^4@Mo8gIg-`L3oQ!O>rq9A!g4Y#%+zf^zF$++!MoeqQtemmZ5gCSbSNw)b2j~B&u<>yEamNOi&rU%6j0Hj'
    'u#}|9mQ!U?{9yWB4Wpyx!6tEv{)9zjPM7Epo~Cxzy$U~U)h(YxDjF1~yg>kLg2#P5=9-PA`5WTmSxYJfRJ127AX8+BKe~_8P_SK('
    '*%_fGP34e^_Jmbp@|;A+(Y?Y``u6-7uw-8fsOV2v-^R4B|5%S{9c{Q{4TfH(YDuM#iuQyBWX|5*cpV2&P5iReu$4WL1S;ASmXAr|'
    'Req;VoYBS}+}qaZQcEg@RCFiE+=+$lLC;j5myblM0;?i3g%m`0!U8gBwH;`6|AdI7yQ3TJmb_S(sT5Mtp0I?>DMa4j!H1WreNJ`#'
    'Y5>cdO(7Kx3UY%&JhkBd+6&{)F{Qhw56v!8uO}4^3X8~`c16NdJTP<_=m!-w>8%t+3aRK&D5Z|fi;fYQqCd*k9QyT5EK(_?qC;U3'
    'nIcR+o$cP9o%DYyef27JNg)+23X90JC){-m-~qNUD<5*y%+M%O8Kj~`VG)`7l?a~S5NTdR5p5`G#m?rCiXMe!WU@?)16h0!f2t>F'
    '_aXxHiZjhY6-^4u$h62RdzkO{3Big}c1^m~1WKXGK!wqzVAB*fVE!0DW|Nq=nci@6r5ogsiY|o(Wm*N9om7V+3TECl>0*hQr72QK'
    'MVG>oGHoQHg%`-U-~I@8OHOZksMVTEAr)N;3(6GGt<BUQ-*12VJN6E#ih`;tJqJ~^DJ&^-4Uh-LehW6tK50n>1?P~8Hiac+ij}b('
    '=NPH@YhJyytc<KY+7wXHrLd$-{ptCEOEJeBSBLL%rC+38Pb#_;mXs;VXt(6tyXkx-33ErRIm<buqDx^(naspK6ZYPt9V#bwHFTRk'
    '0JuoKo>Vj`EGcs$PdUznwI;;vrrQ*;s-aCGMbV?Mq)Zim9_x4L$bl;-SXWn7k>zlT4#hG(!Qgewr+U9hV`o)xlTv;;q@qKyypCe*'
    'f<L;Kel*(fnc1Myuh-_6f+|`R7L_^4#z`H^$b2H7FDg~*wWd-)y*DVPM`bQ)D!q@%+vef=^HlGx*JUaN6h%-fCfLRpc{R)ZF~uGB'
    '42h0y)ZUjuYDQAtH#(5cPSUg%0&kj^ed#JvuP4Qkls$PgU0Dw#`b6;BO+BW#Hop{55<x9HrRFp_A%~eOf5NZ&=wsF^VhX7hNLf<m'
    'v}`L6wmN8Zn@QyvWK{DZQb5fXL7ASg`mhHjWJu%6lhegj21N>~9!XhQrv4%D;JFv^Dj`F_KI^$R1=WbC%%RY~Y3wex`&G3Rswb&e'
    'p>ql*h?vZsP<B2aqMwQ~X*$Yj)te>JQ&2^JVh3gRj;HWhV4F6z1^R&Mn$H0h{fV*l#5R)dpAdnUJANXFeU(3vLMr+b!y88j?8#vF'
    'g?>}l)Xs`SQm%z6164FAhSy|T=*wxAhYzLNNtvqAs~bCqRCFjTDpR=A1Je)ZzV=#IB37Jz4you+SW>2pDf3*FWCC&SgKGPRMe60G'
    'I9e2zl*yy*YV*S@BK2!}xlDq}v(G^lJqpXpl<SG`V9?^9CSO`Cvks~hQqiLDw9G}<)6>29v-nxhixa7o^c+ypqyVeQMA*~x4x1lc'
    ')4{q`TNJ1aP|>2Wu*{_g)o#bR1Es%fdN<C}6KzO2=_#b5M}gMkF)R_T`9JN}_PgGDRU%e@Zw{zvQFvIU{_O2M%Y*f2@gxe?ZL$uj'
    '6j0Hiu&hjl$A0Zy2Zesc#SwDV`Q?C${)A;^>LrQzJinLv4z^l)_v$stBZpM<CoC(Ir?he2`=+J!?5VKJ-?xg?%SlP}CoC&dKk$MF'
    '<zu6vUwS*OR<DRTq@q1xS($>2p6iFFVi(!V&fc+V_e&ua?Fq}uR9Ux2M9jnXN1eU13g=k$<dK6a8Wa|mDXsfh*h8|_TnszbRA^RQ'
    'Tn?$|P|R*)I{c<|*#2;gixlYvc-HopLMl2G9+o+~>xF*b-b*4ZE-5UBqISO&QqiKA-Zt80`px$c!UGNWTx=V+T%<BcMUTS5GWF`a'
    'AD#}?etO~UyTiBfs0^|kRMDldv`kfV=!@zwa8zY__qo^up_fCOLMplx7M5vW$T~OpGWF)6BKA5`2UQBG6<rF?%4|cdhU}1UpdWAt'
    'KX<~>>v<}LRCFnrCfa~r#&e7BfaZz<+U%`%zZ6o@rSPmwzNX|H))d$W^s@A}sxT>sR5U3(D|5=DDhi>|$1Y47R6RK<prS|NNtybc'
    'u4BG|RZ(=$H&~!LzmBOCQqiNZq|9aU2IppMJ<DABH6W7enoj{0EecP{T=r6UOv+qMzcW{UV^qCM5~*lWcv7YfKQujYs7$iSU^}Bx'
    'KVc<0=b(xfg+*oBBujgJ>W6OZ$=8&KwrD}QNhzqJN3nLN2jMx{Pi*vxKTx$%p{5z2q(zHjdG_{_evWh&X~RuinQfG!NC6cc3d_n|'
    's^<DxQLuh67xK`o=B!9%kctk4)r8sL2Onkx2Pd5_Gf8h=b(=~76%C5zg_*;>*A9ox4<`BRRIg&c98T~33En}OKyyF4-(yk{L3`k+'
    '%vpz23Mq=Dz(-E4orQFo<F<C{GYQqpd=9A@NqN^O;UOn(A<~{Ur9!^^z8p#%DVaH8Qtmu*fS1d6d+IGEk;6$Mr)5=s!y9d697Y(y'
    'ZtZQVI%bK998@cyvXso#Rm<V|S!!8~saGu_xgaT2FGtl|MCIelsgL8Jd6n>2&!}nnQEDQERF9-+BJ{b<3BFxa*zj6+bK<C1`5aUu'
    'qGEd;Tm<Lsk8$_7iDgjK%Tx|2h@@0hsGi#und2~TiguMV;aD}pEr(PzD27k+z+FLcnBTkpG|lTrtXUpOsG>tLY>bgyt=9M3ANILZ'
    'g7uVwRisi#MTf$&GDU0yJT+sZej&04vvihfo6jK?T?&iJ)bDin?;aj|k*`LOxvF+H2UN5vhS?MYr(3TFm%=7a`ETLHW&GukiY|pG'
    'WvWQ;r=U!1Dh*vvAYn}!P2n`6MPWIavaaVrJP5z5#@Q<V>S-#6RJ167blW2of6+J`QnZDi{<}G_PN@`9(V?)I%++=4J9i>tZ&#UL'
    'NA6WTmO?5z6qb{@K(D>W36?53u0Cw$KA<Fe3aRK&pv6#F>E5}&jcE__5QRFh6h#WDXi!-EM+1U^$0uJA@|+&A5n-+B{8C6ogJL`6'
    'x~%Z1{e*Zo%=d1SNsCklsc29jZ-4eYZ*T1Zze}tOy9|mPP|=`3GbrS0MLZN_E@;<_ugzR8QW>P8L19su_JQR=@;2{D%S%>+a*{GY'
    'L3AiQDO0Q$=M)pHr`Q<&CW5Wr5OX+1e}c@P;NI8=BOaWjD=UO&6RE8)g;ew>$kjyO{sbJ_<*!;7m|Z_+k;)(y{RxZ7wCNnars(j7'
    'XgTUT(X@FiV4-?Fs%TJ5&!8B~$L-yNQn|5HUD~S6FNagKC#KmGa*Y(|hw@n_pJ1uKSGZa{a!^Ho!c*#JBR-bT;%HZEZ=(UTNTra9'
    '{={S}pH%>K%_YAJHZXO#Up(|SRoAH$RMDZBR+QOgKWN-9D%~H&+e#Z$o^uwd=uog1W$Hc2+pzCtx_!J|=q%NelS2xlMZu;i8tKi}'
    'CHUdNcSq0m(V)_jl2ApL!gDh9GucVi?;G8)*(SNGGH0h+RVk>VO`&&Mdr0j4m=%WuXzu-Q!Ke{AIi#XZ!S?Dg^^e7)Q15+{T6$!S'
    '8j+JjD!LTn(>$!(ZMpw&LkiliN#})2>U}DQRP-stLoj7G#`nWPCCJx^h}CNMOCc3)3L#g2PT`Pjg%-KJ9VHTlRj4vhMV~@UQm|fr'
    'ZTpk^g9><>ODmQ?J4>ghkcu{isH8wzZJqNMNff5<ehQ%av2#d8n_`)y7)xAI+~0ix0R8Se35lxOIfE2Mn_`)y7^o}eRPToa*;^KE'
    'n2vDmekr7)OR?Ogn7qgC-D|&frI0P3d{m?|NJW!kv0r_?H>w@{B;Ee@)IGg?LB*Qppo%Vqjppr&!PB>IW%?Lvh>JgA0IIuh3aR%l'
    'g;-uDDk9m>iHDQQrk(Fx)9z0sh3fUFD55e=VQ;_3RA)=0*mW<U*RPuBkwa=mQf5>1wX-Jr;h^$XVEu<NLD}g!q&SkYgsVwzl^=pb'
    'pwq5<0UHxr>Mw<qL{e5vVQRr6UJjkU)W+PCr)WtnsuWZ!pz5kA^oQ?sEKkSLu7hf{DV3X)LTa{1$|MCX?@ByOa|4^?ySox)Ra;^X'
    'sUAs@t*O3D{Pp{TYV>{us_XwAW&Nd)8j+NTX6}yb`@Td(gt5n6GEqs298wTTE&GWX=E?dStxMm}zLp+Wc6tt}=u(XCcKV`Y(1Cs<'
    'dQ=%H9?X^RmqRL=6rP#caNx0jUXQY!nBFF^NQH-_kcuY7@FvnWPG=Ndl$s2L(tRnEqCqj())V`c_GC~<lO4mj^e$CSdJ3s%P*`B5'
    'B>;CX=DR0_vUT6Yx$4O`hg9?@z|@4^X}y}|zJ`{%?BtLoh}G<tLW-k50j4IFfEP&|o;(13nnX9tX_T5sAr<`zi_9GQFFE@Y&~^va'
    '(yz}4BNeIFlZyTXm_MP?&R=}*2M{2xKS_+E<E(3M3aV&OfH!*}$+<RZ(!kxgtv^D&Bj%8b28Cs2c9NaJEqK!D9(AC{RmXar%0U$!'
    '3hS{RdY_^PkMpY1cMP{+A9eRlAr&18w9-sHh}v<MFA`i6EvJ6S3U|&S6<rG4F2#-yX@~7k?@@lG0vT9IFFB~9O<}Q_)~ti)Ubi9Z'
    '>av>pv)7d>g;caD$TbC{KYHBnekvz+4XxF7Rux4CD2XlwZAH<~bHY%+vFSx*G<np8Du+~bDJ(dXW^W#1aF{tSy6O1_sY{D%QRSeD'
    'HU*iaP~nZoiI%85n;%qQq{>cDAr)N;s~_`v7F#fKzyH~n=KG)Bt?K0?2UWBwrb!C@5Anf`YK>hH-Ig+_I*Jrh(WS8LOxOe4j+L3T'
    'FH|yy9%?1iQ%FUZVwy`arTciPLv7NM%W^_#v;3qKQqiTDS1pB;y1!BJ6>{3GIH{b!98%Gwu;fg8D%=?^hc~Lud#c;eoAM}~o`Nd6'
    '6l{`0ae1=`-K)Q;H)QuIY#IpF%Wnp%6@3c!22$aEKB=a4&AXMsJS&kbhg9?_EILz2*9)CU0nEu>_oBb9dils96>SPj&ID+59u<o)'
    '8T~MuE-Q*u2B~OMu-g=Xd-DAK%Ln%{_x1<6uF9rJK^1KZi_RR&zW#I4%S1h&JPpRs7OE6f(WkKNOv^Elb7v0n^*qvLD%T>FK`Qzb'
    'mYq39-}6P`p#EeRTmW~dgDM47v?(k(bBwl|-41p-vJX^jgDZ|P2~>0`EI1Pgh>wG^(dJL>e0%PzCWhsZiYA2xXIdlg4x>E@3c+hy'
    'l`<$)38+?!CdGo?Mnw<i@bRCz7Z@r(N>QYsiY|qIpSHK{;g)<~Ows#pu_qf_P$yLispwKzbSBd^xUwH4#keYz)~*{YQm-c!T?*?9'
    '6?-6D$NLnw-SXi9!_}LZL+ZUru{=3b`ZW?_97x&QyuBv&&Z|7y6jBsPfkk2po`*04jqXZ*O~S!NDuvXHq{!ZVrB6B^Q?OM!ihi@T'
    '`K5s32+BMPl?%-k_`#%r%S_j;a#X@|4l0SLY`jOeyRAv^d1xfKYziN*Q+MAKR4bse;7sVi9%gW`DFh~u-A2WgoSB2Fw}{GY3R$p@'
    '7b*epb~lxP*lQF)4yYbM(WJS!d2aA+(I0~fb!Vv(uvc?V4yh4Isiv5ng?(V+y{6Fbz3*97Dk@4+q>zG0N+ku$^+I+3@F6tzbR)zu'
    '%c9L86-^2Y&P0*jn0;?j2%9-!ZzB`*Ll&tFQqiQa;7t2Q)`N&sq3N||diNSNhb4zpG%1Fg6v%X*Y`MlRPU-?GK1xxfkcuvag=az+'
    'k_QvRgt=XlSoHs>ezz1<(WS8TOzI8p)x?~;ai{55L$&dxFpBns<!0)A7TqPsgFtVvBg<5JaIK~cQX{$(o|`#k;d%V1fX`@Ii}b>^'
    's(v<wRJ127H&fNbed82aA*MguvtF1>Mx-!`#)Kti!Xy+QD|P?YL04j&6$N5dPo$8F#)JiCTA`ocp>D6cgxG!bo-EK0Sfo-&MPtGO'
    'GqKt5spFbjysOdm62%=$OQevB#ss=C(dNTCmn$=5k#2wWCY3@e8WWb631$%;pE^+H@FozOXXHgqMe6mWqA_84nK1g{<<N?d_RhCv'
    '4KYuG6fFr$%B03SgyP^!tSu(i6g{lUJ5M1MjR|sNVtO&`AuUAtc=F%smey0UFM||BW5S{`0VkZt8IfLgcOg2O?5HCug;X>qi00Jw'
    'LUfdU`jKf@xGY+;9HbOb(VeiMOjyz%q@;^}Zm&ZJY*4)&Ii#XHVL_Q-7hm$ks@vVKHw%li_BRHnU8r7<D%ukklnFBY#py84Xi>kO'
    'q`ViTNM(?U_JjpxlFf4SVe^Md^tlTsn%=CqD$#RLMSsGQGHp_u?|*!F@fep*U$C&kl5$W*gTkUREwn|BqJpek!b{6kT{NlvFNIWe'
    'DJ&|}I(y|@jU;pVtBBr@xyqY!Kt+>+T~c%&cr3+SGP-wF*}tiN(G*e`O$y7&#3p<EAso&r)f;|2)-M{l!nadUMVG>YGGPvG$0rYJ'
    'up=l$Kk2O2-W*cVrm&z)9J5(#@BM@@{V(;#>*thU#RMmjiZ+D>WkTPxEaiScERD9a4;n2hk2Z%?bSW$+6Gz_<zI?x?^jmX3Y4%ke'
    'Q%R(vOJOmYc2Idi=KX!D>qp-Cw0-tkQYoOKM`0<MXcOhn6T+yk=I`?z!1PC<%0Lx83d_j^LGm@l^R%e^&ZUZi7FQ`n3aV&QSW>25'
    'k-$~@efF%)j`i0>v(qcjIfqnqDb{}Y*vf7lw!f<|Q-3e_B9%dkqDx^xnP6`oE-6kj{kl;BFKl)9O+gi13fs>RX9v%-zc-Y6*Z<ZB'
    'N$4l6hWQ*+(WbDZOl;x>vg6?VDPn)2!pZgSkpn8a6c&^TqJQ~RmE-Q$cK{jHgq$2u?@Wr>JSP(uv!yTggPT71s;1z!xX!5@R1{Hx'
    'pHn-3Y#c6B;!&FV6G1&uC6Stul)ZcW33SH8`@H<A&Sb1--yBjLNqHs3r55hTc@VU#<XO1S>c!4MB@vam6ttfb1rFOE&a2T4R=Kn}'
    'q*fr+y?c~-rxzToKcTTbC!PeQ(=$-b7Ezg|AlmthFc145El>3W)X!LV-xO3mq9PmPYnk@P35j}^U3D*O!mLYG4yqARnW_*FI*W?Y'
    '_Dhz2%Gs!nsuWTXNqPMCU{5v0`$eTJ_dFxAY`@ZfDX5}TVPTmh)cdhQ(*)u+D!L^d6sZ(a(WvmSOfeO>7O%thcTJwMRCTC>Duq-u'
    'Dl9D1g3|ezL`IgC>0y}`^imxkDWIZHVOg2D?6k7Y@8DCg%{w>mZ;CfD7OK~yiav#BWfFsz(jIodf%}{qVqL#QDuYz?DZt~m=f3<N'
    '?!H$dPqTw=RE1IoDULP;xT3g<$?n$_to=f7{`;6kDuq;ZDZpI{cI|g4r3HcAL^@R_R@h4psc2GIZYFtmjwiVsw!i&KrKG7cL~=+)'
    'lL9<_`x;h6huqBnt<R)tq0a#oEeem#T&>$7)DI$k?%4y4haOdRpGqMWEeiBpiZ;EL;!vNoWM`$fzNBg?a!5ss!eTR#c&8&B`rQE6'
    'e)pZTQ7`S6f-0I6=qAOO{rv0;tRK(|lmG}Ti#CN+^e8+$Q#UtL>2kPF{qZQ;;92dyIi#XTLH_E2_V9Gn6a!Vydo7uM|5fvxf=Z%E'
    'Vd<H~G~M_+biC>9xcgDXgQWtLa!^H=f>cz@#_U%4NmQ`@w{9re_|ejTDX5}NVF8*n#S`QX@xg@s%1wvHol5_upo%_)C1_egl9Rgk'
    '{!nO3#9`i2Evgh!(WtNhO(OJJ@(m6r)$HPHs$(@$C6S6og$HQT>JPj>z^gu;xp6C(Hic9)Dl9*fc<g75`}uoB*7fFTSEB2$eA*OL'
    '(WscNDf*Q2qS9~0H^`1-l}u0J6m1Gi&z!w&Uk^_zop&IuWo;Iz6jITqum@jNbL`>4hdNsC-u6qEkIODp38+@IDJ(zJD*F1lIuk7j'
    '-;G?xUL9{aq@qh<@tG?9c9YaWrO&w=J_ML6mnVl*G$|}TbB^BJr9F_MA$wwr{vUOnN&yuu3QNyay+Ga}f1AfmQ+nTbW0e%uM36%&'
    'S`?O^NlTh*^aCmUlW|W774n<{DmoMvo;iU4&J};8E6VM58YH#(<&cUFg{5cG^rwvG{f2kPl%+AcnIs}mB0U9FbSNx6bE^FI2}k#&'
    'Cg@VyG+QN-<$#J7g=J^zU8;E{#bKto?vT5GzDZ>VCy|O4g=J?>`{~aS_w&y=H5fq3OnN;j`?ez~aC>$;>2&YUPWr$8d6p7?DX5}F'
    'VcD6CyPQSlJu2qm?^7Myel{*tuSXR<3d_!<HJd!r>67<(H^fruDV(B3VbPhic=U75)m*9(U8GytMUhG&_1>cBo}IaH?X-zo?dLLn'
    '!?ZU0x=y8#qDV>=#j;0_^&pXKwdsup`VDYBdgP#*5tYiir5x^cpXhCi{9-&B^;Wtsg%U?f=1=J7k#oZ7Y_o577R#Q<;UtlhY6Dh*'
    '{vb10FUGsU0D8T4<ZxPnlZ7Qsm_B^|5E{a^?)DkRgmsO|Ak|wW1$N^_Zwu&$cd5?1a|*_+7Pk~mJ#unqLU#S~p~ZL7b&nD~te{Y('
    'pc)Ys+aX1O?flx|cV2fUm<H7Dmx2l+sx`@i>-&zaKNL`07pd8x5)N`mMRQ_!1WzMAhg;Fu<M&out8>{%DWsw~VR4yLgmz%u|7-93'
    'b}c(@^nRbm$Zark6|4BC{2aLo5C|Ph6FZPCMQ&k`0Q2r4S*tzGIE&|InllI}NDV<cr+e@HJ5`JPNPbLE;cZH?jH-=zYDh(G!t65X'
    'Fj={S-#7Z=!u>4v*b|#n38|=0m|rI8XX-N1yNJ$Wrg3il4ARnU4XLP5m|doRAmSy&WE7h~*!|S?_QTmBMG2}XQkY++h7<b(F-fzj'
    '?dJ~nY;m@N6GVw(c|b9a2X?QRWbM0o6bEs4`PG1m8pZNW%IMitmQ2k%+&e+<$sU-mAr&PGGt1ONa&B8}*5|TrLnf2C95kv5R8gW>'
    'Uh)utd}x%Os{C29<#gg(LMloWW|yhy6!_xh#M(vhSw1yRcR3f4iV}s{WeV!uhG{xQpr_73*{bZ0(Mw20iNfqMSp#q{BxVhn>&rs7'
    'vrhM;myn7Qg}G(Ql<Vru$(ef04Fu=z>E0SrQKHy~Xo7yGX`i2%ub(RvL-e!hgsOy8R4B|XQ)9pFH`{xUvu#|VyU_#P7Qab-b5bj6'
    '6lRvmXL`8s4vP4!ryujQ=2ychN)+alsZX9O&%T4wclV5pt~01(^b%50qA;UO(HDQ`L6f}qabFZG&G%EPgj7^0Y+!@VV$KzcK2!e&'
    'q@qGWKH<Up_2MJj^>MXXp#|MORst!C6XYG4;QkZIVpxHDoaQj&*$yU3NJVkNEHVW&qQfkaQc}V<-Vi0$(rgK-C{CC|rhfK!;og3~'
    '{SNNCZd4yqqt8?csVGjEN9K_mpi9=~d}SnkN{O^lQVppnPMAlgpv>b-SKbfR(2q@gUX+Bp-cf>TM|HwHGMOx}NINK|Tr1~|^Q?YO'
    'm4J%!gjr-Bn+F~F+|;_01ANBOyRb=Bkc#qzS!9Yv37F4K2|Rv&%PJl%Pn3{~@`QP03a)bp4c|!#-^H-Zyt{#-hE$X%%p+5*?zFT1'
    'F<W20?po}z_U>YD38|=2m`SGSYsYJ>^ZuZs=lVmOt4WoRiW-G@!DDSoZ$Hg<A2kV&!0ZxY38|=2h#wY|@xgOy$?$^5Ts+#{poUb`'
    'D9j{NzXv_@eb*??Ja^^5&}^zF+-gWgjlxVaML!etl>?X@s&!Lez5O~!uK<Nnp)ik34Zz7=>feV(X~NUyychLjs)kgQD2@jd2X?;2'
    'Lym?Vx5*jX9=!%sR4B|K^UUjynY6m-|DR9Z3(`vwYe+?f!YneG@a}Jeq&x+h^LjR=*JVGZN=SXJP=H%x3bOd+cam0@{dAZi%=Wda'
    'A%&5Y6*~r8+gVAfQ+h*=`<I@nK2#;BD5CPB2i!#+?`;$Z7khZ9=MQI-Dj~&@l+SqZ@Wgk$Kc@z}-Xv(bSC^+sNJ%7R##R&D+C|={'
    'aWqCe-jKea>xf05P5@<2na6y8^UfqaQ4^fyhPk?lqJk8SNNQQr(d|cnw+o?pw$FGBO{i9{#;GASBPrhshS#xgCTB{9lajFo`b?FO'
    'f=J3VifuVnIydD52Hxw8$Q}CCkXn({@#PS1znIkKttjvF11P;?zJ^m&DBLF#$g7Bw=;+U;J#)-lefQOniV6i-Jh9>4eVhpC`bF0w'
    '9Y$Y$C7_}{0d8xhhsss*o%AIwugqH!pQMXZC8(l60Ul4xW&L3@Ye`3OJ9`^i?8~nNRMaQTCiCd)&bPPG#I?LCf_rld={2OHKw&PK'
    'N91}da*|9TxZyqN*`g*@L5ib5VLq86<90e@);|lf9#g;3^N2O5qC&Bd(3+UNjhPawI9|^-520>Msv#8>3jO>Q5`lFw%lnZ!&+Ruf'
    'TAwYXmw<`_h52M!<h@Ql;=8}}Q0z6k-djT|N)+akc^CjsR_`T!S94!^w#|}?Kt+YZY%<THA-SO`&2et8-?U9SO{#)alqk$5^N3lH'
    'rH`pOuATGLryTTCssvOND9k1ESaIGuDBjn&ktW7o?>);x`bJfPDoPYvDGdZ49S}}R9{4O>k<;!jMWmucfo^NHZp$Slhl*f(&>{8_'
    'wC-1dN}@)AR`i(eX`1bE(?f;6?nBljX|uQ*R8gZat4xjlc%<gDwC1zOXdbImHK__xQKLZ9DAv=*OxdC0JLCpdsP98nf-0&M*2;49'
    'q`6A*K9ERzflE#YhVA}nR28VAOyOpkbUgm!B>pqcXV++LyKFm=m5_=$1!+*BDJ`vZB$1}T-8R4UsWv-OFCi6m3U|!3p=?u>-hD$z'
    'BZ)f=x8?$BR3)gQQb86}OuFT#r0M0*^6-!5ChY64gj5tN$b^cy&=1v29#r<h`2dyKl66c`f+{K%JS|eU>b%7ZNtMcAx^;+NgT2PQ'
    '1l5U3g?VNk(V)Rmy(d-6HY0jgwFh?fqXtz}Di}#$_3&o0WGrzZTpKL%A*v=-K`JU0Y(a$y>-iMHJa{Wv8U@bXK}89vC{(Zo6$Fb?'
    '+c*#gmz}09Ea);-38*Mjh=mlieea&UA1R!7D$)8gd(^jxRFo;iY9Ht;R&0_jx&37MwA<7Wq!pzaR8gl8n;^AI&0A^T<8;I$o>J`t'
    'F7-C5Z;vYK6ygn<a6S<K>;ZQ(D&gJYUk$0KQ;2y|=pD#6U`YDY@0LVOkg?rEYDh(y;#f>EaeG2FlOojuAEVL}wfgp}Af+F7BxR)!'
    'T;?>JY%<rF;~}+#TcfA}6*Y?Eo)owIo@BoCrHs!LMi0Ezkct|`VY5!!1UV@U4eu`BJxHV<QzfLLMsa*snDEMVKU|Lww+w*Yw_gdV'
    'uSJUW-4!4fvhNCe57M{hQ}>D-+zm=Kq%e}QfMVmV>Ar`4phw9cRJMFn+U2T*6h%@#LwbjEMlwpUEt0<yEsn1GmXP8|$|MTm8&SO9'
    'etP7tyPWwpwtq+sDT$;^qQG<aOTC*N?ES2ZKt9mj={2NIAT=$b&~VOtGdYPU#^mdsL^7#kzY0=wL{jdXso7GxDUlHFVvn<BjSeVE'
    'K+Oos+9(j|?EEpcA9MRd6Z<Ju0tzB1b|+fkbpI4>Sl*c=!+JE^>Y#+wiln^cVa9M4)xDFV$$t4+y9IT{w}ezwC}7$G7CxR%LDKE_'
    '#1l9*qG)}#gj7^0%r|qv_BM)<Sn}ZWn9?VjbQeVpsi;tZDHIwGj$HM#xkOj}CL?uUW(}#RP=M7u&gPZR`fk3}DTPX-gY+6wQK8Ux'
    'qe1%djh2!H>2PN~)|!i3p(r606^i8w1t+_w5XSNI1aO`48`3w21W}w=R_-|Sc8|Rmd)Ik38u{#er&1-PqB`N0nM`Zi#h197^4^|q'
    '4Na=r;I9Z()F+ll6EHc?dY4<;yB(Rv@=dCQRMaQdo$`Q_M-vQpw*D~N%tw=|AQknAZP5e?jl70vQ`l}*yJTUzk=2lj`o#7Q3J`C}'
    'oJsJf$&@?1nDp0KZwaZWPizY*Y)!WB-pk^2yywn3otsn%si;rbdTh;O!3@)r8lt|fGw*c47;Q#TLn`VMHr#8IZ*E4hk`4-Tu}Nw1'
    'k|tF`D(VyH0)!07%e@RoZt4p(+KIM=)QTbn`jCfyu(>*m`;n8TetO-NR6{C?6sQpe!M>YE-$1c>RopcNG*-_d)_{r<g;{389&AQD'
    'Gui2e`*C;KntHWFTZ1ZU6lfKX#jYkL^<IM8<~;Oih1{e{NJWi;T%(wi^IhgVfV^oP4rAYbC7_~2VTPGYpH=P|l1yT2%;9R%;nOjC'
    '4XCJ4kSP?hd~!e%%Mg}lbbw_^`|#ADiV}tSWv)SYxm4o9XpzUdx*tnVPSv1_8ig5V;?&d^)4{1@e_J5G6jhrr){u%Ch52RLN&84k'
    '-z<Jbf+6eJuY%N$8U<TOv8Q<KhsmJ=9=<C~0}$yQEH$X2NMUxFdx6Y+Q{usY3I3W?5vizBm|dnGD*j*`5sADhUA8Sm?~tN|R1_)9'
    'F4Jsw#!k0n%j1{?djG-fGgU$=iWK6ao@PvGbCN!!?)@kl&-an40To3Gv&&pE@q?*(eNG+kco~eRA5%4?qDo<InQ#_B?(1j0NiI@o'
    '4h*~d{Ypqhk;2R}Nf_SmJw#2QJn2OZ@w?fj1X7eI%qNo-(oP*sc|DEJ)6U)Mi?4)Klqbw5(+o|$kB^gr$GU#@h`hVexr7u(dBS`$'
    'X^i8NU!tS)dAto4(P|yNgj7@~%qVjqpbw3{mwQFS3BKa2U4>1mf>cx|%qSC;mG9}@iR6berl(Tv>{3N4suRbWGQmoN`IyI<(_8(B'
    '<g|)j0_tmVV!Kb~GS8}!cF&rTsRtB$4(u4c1QbS48c%F}O3h4{fPE9G4IT59FrvuFWQlcrmrAn1QuE#IOD<`huE*Da;t0xAiS^9k'
    ';CtKb*~_OpG?_x~exwpo5=k9PqXhFbiYMO<M&m3zJu^^4>I72ekGbc*-sqGxjxpTn4(X=Z8c;MMD2*g!tosOh)sUF+4$Dt4Q`2rn'
    'Dj_u^DN`nn1`GC>N{-aCh+c0=LFhA8LJA@&4JbB=NIo(kEbuI^)O@!|m5^GIlnr-iI%^Gn<6W81M0egX9N@v7O_h*}(uDaQEt{9h'
    'by+6g$2X=d;clF$Ar-X=Gst9^d#4*=85rrVj?kChmtP5}C{Tb06noHnK}wzlqxziEl&UF}P>Sk=d1J1%H|3577VEiei<(f)zW7Q='
    'MRmfwG0B#4q)nKbM#<elI!^B_F9H?CiRD`a4*Qnt<Vf)|Cl7<&1-eOnb5b1D3G>I?d&tT!ze9uWCRp_G?FRMjK}B(5c`PB`w4Aj~'
    '95?3C@En&&AE^>jQJgS$Ow>1Jn+6hhOx7bct}+YQ-Lf^LqBvphm`g7|$B={*N9YMB+P=OBRMaNS9COLwm1evoavRX+oUoynft8So'
    '+Ju>7+G_ce9Vz02+ox})UVgo^vIbPtCd?aiZ9OL^D|T2|?$bBN6m;rjs)SV3C(IjjZ#+p`^V#4-*K-i9zV}KXMQOrpF}J>S`oMBR'
    '^!;56zttL24X7wh;Prgl^1*QY#w$&r&}?0{gp@>S!X|GvEb=3@-=9#nUd6)=Jz4GETtg~q6XuF(<#^5;C(fM5ZMJE(cXJJ=s7#nO'
    ')fT{+w@qlIaqm77p1lLHhE!B0%o9^nshcDvCI<UX-khzGNB4}CfQrflxj7Xar;s$vP9Qfgwa(g}=%^tTl?n62B+*|}J9M@}FS#j_'
    '6{VU~38^Seke51^J1z1%DbUB+4bfR2Se^Bjkc!fT^@8om+p34}#vnW<+*LTNpPe+R5>ip0FiT7ZB3r7Q_$&&DFCjRrmtU)WC7_}{'
    'VUC#gfn4MIWT*1FzV&A!t*#zckUCMHFh@+i0KK#MJ)7e7CG>NBw)5T+Qc<5ULrfHX@}#ZJN^0IMh<6`Eld2#U6$&%N)QEz-EcG5w'
    'z_Tj{^xZe75>8Q{Fgr}mq|m#OR?>n#wol;5p0n#6HKd|I;dYouli{5<xs0`MAG%+GO{#=c6e!FOQ=d}HO(l|wJYL_4p?9bB#-bWh'
    'QJ^qC%sn>q2Yv?T4dUa6b7LQ=5>Qc}5HENPr;wBof!FZIl<i?@&98=3)F;dn(;Cqk^UH5OW{d;2V5hw$q@qA!o|tHN8m2oC$9^nA'
    'CfK&hCiU$}=@k}9Sp&u4JGDm1w#qO4&090zPNzypMS<e@#*QtL?t!qqqI}J+NI#q=RYEEX6z+*RJn&1L7TE&7+XKz3+sBGXMTx>Z'
    'G56HCE^S3;{XYDn_|0}msv-5YM1kMk1z+21xso*c>0xqIwtu-pzY<UwL7759b4VWgO}K5)6X)o=uLcxFP%cpDOW*0$Cr>h^DRTQZ'
    '*Kp#<iJxZlbp4^;e01HB7TPFwld2#kk(60a5G)#5Szd#^r{KX`6W>;nN=ThR%8W75o#@ln+4<%2J7${Rpb9|E5kXlrakjf&CgQAu'
    'yj!IPLhmc80W~8iYoE~X+UXV?UWeV}bWRONx+STG6hu;Hv^~vEB6D-mc!F;iw1ySX;Y10j6+xLe0o$KIiG|-rIqrFIY)fUY04pIC'
    '#R;>;L}tG6cXFi6=+CS2_5n7j3Q|#>Fk4JLD0f~o3H;{S_t<S^)*^ZdsVGmFFDBXg!@MwMT@!b@%~n<RbE<??6e!Fa6NSedsX(De'
    '?^?brq;_1a0TuNLx5gA&k}vK6S=SoVKiUo`N=QY0!n`qQ&+EMN{+L>fx7JG=W_P-jkb)>sm^UU5T*)%!MURF=(v$K<kfJzYzL=!x'
    ')JP`=fX`hmH7w|8xvvCN)F#XplQb~rv~~#mOiRtt#ltF4QJXMVOgtN0bj3vOgFAlBM`nX60TrbQbH&6nX{K9WdC#>x;iaYVUq=!p'
    'q@pyjJ(3tFHyyK@)bUfNvElAf(Hc@wnlM|;Eg&D5j+xz`n~truu|=dBQc;@N9#8DMR5TeuJZCy7(%g4<<X1u}Y7=%?ko6-usq@Z$'
    'DvU?p;qJ??1XPqJ(4~oWrvX;dGP~z>Z`N1d@k9lv6_p8GM5K|o?`nIMCyuSvyDJ8UoEG~^Kt*)|-FZUp)%PwVja%pts|lZ$kxD>C'
    'b;7JMiM$arD@mD+x+lHLtIcMMNJV+VtTAzzKAw7x5#0VhbEO@9`PGn$`UH76!Dr{K6N`DHlRCWm_NyTk^$GLF)R>!d_j=x%^t$hk'
    '?ey&ky#!R0C(Ig?HVn#JG4(@{d!z<Zx1raNit>b6V=5_^C*DVJ^o`wpZEHK~=Jf43MRCH7F)d2psgu>-S47u$DJ@RafQsUT8Do-c'
    '+_^YmyDWWS(SnZqib(CKPVmypPtR3K3g(7q^xAeIN?(2@q@p}w7oRP^<~*FBN0}A2$5mH5N=QY0!i+J=y(iN438mvR@`BP;`6^OT'
    'pD=4o{cD_jNjaWpo&329M}zwIprSls&X`019-$|oVB+J6PAq+-N=QX{!kjUg(RQaK8<p8Ml)LW)!IXYX)sTw%ggIl<0C5k~cTx<r'
    '?uIG-m-aBNAr<urxdFwJJf1i<+Gag|gZe2|!zt<$;)5LslG|AEo_7L=g}8I65>Qc}Fkeib@N5uBx?CQ8Vgk(B`a}gOjQWK6ViIkZ'
    'TuCAjy!ylvnce47LMrML=8H+b*fm|TLin61fx~_K)sTw%g!y7>s&pvdgQ%%7TtkCp2Ek3Lf>hKe%ome}Y|J%38SBP|lcfETYCwIh'
    'PmtSU@?6^*U6RXBqj>OB2WQOP0YwQZjHJwYV#z-FK8UR$$GS6!Fxzmkh7?6oUhv@M=Jh90lH`ub&x5mlcWOv+BxL~w>|61GCiU-$'
    'ltfZ@wJ3Y1d7@CzCcbTc=4sHRDoC9`%B(SYE#f3VhbNlQq?cNmEulmsQd&3Z9a$PRJkt`Iz-?zxLup1zW;8}{d(2-W38e_v*j@v}'
    '*{h{$NI@iJ`Vca%+;9g`(_J6!*i0Qs)R0<{)HSh!4~DPLoZ{sWJZ4VTq$)^7Z2~-!;Dh<_HSu$%rXH$x@T(yewF!5{96l+UjEY)2'
    '(j9Drv$`r(11d@rZi>kOK79QSO3q`oc+BjtNtKX_(uA2}GU;#oLD#Hac{kyO#k~QshE$X$%oS6%IZI^C6Jnn{+{<r|F7%dwipqqU'
    'VzNel9xPCDr0A}TSjdw)o2nqiQJrv8Oq7NBvNo1ysMmtO*#lE0q@qIM&X~e3pwIUhUG{Br5rw*_wWJbKQK48&w2!}>mnYO=L3&on'
    'bMIEEffU6Fv&B41?vCDf!Eb2N>rS(1tQu4WsHjcsNn{EUWSz~j=60W}2?E?Z^=d#xX<{#;#RE^e^;(}wa7$8y+7z=0RFoz*>WPB='
    'c%8q4qU~AKnJ~ClwbhV{(uA2}+FCD{``-IoG@S6RvU&6sP+xu}q@pyj4+-q;a^Kri3Sf=p^EYM<w;EIps3=XK9uT;Lg~_ylOgl^d'
    'C4>9&t05&(nlM+)x%ZmsQa#bo@XmGk+KwI*Eg==93ABKM2d#8QUaXj95rr(qGn-TisVGe#>+OdVeOBt76#UEH>xtZ?N=QY00*{`l'
    'A?#-=$$R(X;iqrCN2O{&MSX%ypV)}jgZ|D5_PSg5=rDAhw+2+yC&=`Psb}f~ePYh5_-#4Rb>13MQJ}DS-Z|&pewdso-sJ12ELi7O'
    '(Mw20fx?{OCZ!tE_OU~&^>!`322oTdNCOGI3Wd|5Yq4QAxf$=>L*JynJ*g;7utpL?pFQ4X^j_h$7EK4G@gZ^7q$)_AC{3`268fHH'
    '<OwgF<)~n27~gVV4XG$iu!a&Ac|)E?>A9o3R*I!b6_JY41Zya<&hbSM?+vq}n8;_PY(7-__A4P3r3rRv!gblCd<geIouiROOGq`G'
    'qB0?-OaLX_EoP$W;knrA(GJf`AVpO|QzQb=nEPboYy$HM&&eF*8dL?SC`yP%5<`Yyx>QBr%PLV6LFoR=5>io=5G!)%clq4mdx_W1'
    'lb3;IYRK0m-V#z#n-J3`c)?6{z70J5DYeHuvyW5>s3=Vw>zY8AzMK4AL|VhQmmij6BZy7v+mq7YGm<iC;&5U&q1T<p?yewhnw{&G'
    'K#Hn_ex5RU0%T=)jSBJHi*La8sl_Js?MX#d;@B()9=^LBUXK*dXNR%|9Ncb5HKd{{aZHt%w)!yhs+WbgVM4D#S}jYIfcjdNU^Bv`'
    'iI2lCCfdYYzH;iAg4;)`gcL?n9!tm-^4`lFUNm&^d}xN;XR3r0MN%$K@WOP1yv%X7Pkn6t_<J{R4JnSKOq|dczEg8q*?Jb;i)@L_'
    'wl%2=QW8n&gGU)4qzk14hg`P_(NDHPm4G?{ltz7I3wI%}9BRnnPcjYGrI+87fSMzMvRcPp+a&ai)4DEW0s=iCTEb~YPPRzTj!B;M'
    'ZfkGx0Z};v`syno1(B3R6Uu6z4|G}NTj#Y=(gsxlYDG}Tdi(Yb(BF%P`XzUNy#0Hhs1i_7odBN?1)S-EHrZy`?0yrQ+6TA>R1_z`'
    'f(dh@+Np_wAX}|t;sC~OoG1YmwF$88Y+D{LiTAq?K>W~wovbERLMmz#V9~_53Q<BzSzS^Z-*?wo5viz6SQG3#ca5kdE(ZIK3TWab'
    '(#=RUq@p;nES@l9g1i}N+rrl?q)RtWlz@ULPArQj7W1<D;kz(<Z54e62lr}HC8VM}u}q$rI4O<%wwpTJbm-Y*cU7tcR8%LH#}ng&'
    '-IM}zT{leHL%5Gr38^SgER!eBCd{3+mwv%~8j^Nry(OTcJYjB_w$p1RPbicd^_i<Pw}4(kDQXjLg{i-Zca6Qz3hXakV@$m}z6ey*'
    'Cd>-c%7%RMYCJy{W5OFcmZ%{WwTW%xp_)vw3Fyfn@7kVe4U|F9XR3r$)F$>Z`*Zelm0W(JpUb#P)56tl=q040Hep_vOXf|aC`ozI'
    'pJa0ppI#|YLTW{E!n`oo;^%66CuP&tx4#ch569P#isFQMVJ^{lZlp32uZPr}fbE4@Ln^8h=7qThczoaoCwH5a?1j{QV>P6rI)QFs'
    'weGlX(m00K4{q$)@<bJ>C{K{f6USEzNE%UqCBA%?sT57Bf>e|!%ncI>y&);_oe+|F>%lPndDf)9J*lWsm>VYOL0fq@Chc*1{O~rP'
    'HK=b7DoPY)hPn4VAIyp(@-;@%SH4@GEddo33bVxAs4qOf`xqWmxW)Pb_1#xODk>Cai3vx>#qb`ZpF4tMEt%XUj}lTlDirJr#kxY#'
    '`;ppk?M=^^tJB^ZP*I?8M@-qYz2csc@|AmI>U*aqRYEEX6y}I&4lQS=O@6F|U2U?ONn0A!kct8YyFjtm-5c;8qMxU6cA^dTHmM3y'
    'QJ^qSOe-TMkYteGdxJET@2IZ?Qq(5g6Voim^176*ioJW>CPFK-HKd|8VWya{$1P<fjV=TpZwwszUc1&?LMn<AW{SDj*sh_!cle3s'
    '#qJGf>s30DDghP63G>9n4F}SluX12``vjlKB7LMvNJVkNpHSQSX7l)bf02=(qlpSk7?p|R&6soM3)=A9xejy3T<wZk11c&L=h{w)'
    '<fGRn+@jYP&B(N~s1j09nQ&K34Zg3G_nt}-KE`khb~j_zkc!HLxneH4`(CrK&wP3O8MTCZk7y04uayaLTTG=VoEyXOB5^~5t$|<L'
    'V%CttNJ;|=Lb^-XUdxAP*eVErlqs|xYpWqek(AXs_VA$r=A;!}9z%5Q5WR*IM^5Y^`dYb+r18e%BC=_9Gg1vHiKHx^ur8N0nWFW@'
    '$BIq|enp^805x4cXZ~Obj*u+xSAsJ;mns28BZAUsV$XAZ>)+$PWBn4|hq+84Y*H1ZW+Y`sRx~8{ozQ4+_=#HEYQhbw0u)407Eb8<'
    'f&A{nJ+7O<#5uZrSORKAP`>}`Hq2()tR}kiZs*;ji&7<^qBsFw>bOoWNvY##RPIe<#~eobNR^O^;)K~^Zjnj(^tHx?a2H<h?bm0M'
    '`u3!vIN`pSdap@(`5jw;bIm6fOD_SdAr<8b@OT3DM`voBJAY_fBRcRa0TtB=v&GynrWbOP7C*WMNlFS&n*!I6it>cnV&Y`>3gY_+'
    '&M_zS+c-wjjbkOGIO-Gj+|vALKyE#8MBKzrUpI6-Q3EQ<6K0F4N6Ih1_x1@rQG>5X$ec7fo+u#|^$GLEM4M_eA!#St?3Z551<>V='
    '8d6c9Sl&JXPAUl~4$j-P1bbn(Pt=f#3WXVCt}{8m{WO7MAHlJumeA83HKd|KVdj`<-Q_-9_<p8%-@JDtcVVi8Q`9HS8*^(sXNKsB'
    'oxz#M#}5Y_258N%gjCcg%o`IYuARC!p9xRv7%75|Cu&GVfx^5oQ6E9BB)u0r&X=nESi)VJsv#8x3iHOay3@%=diC)n{**#}Sasl6'
    '0ZO7iVb+*voUDK1y&H3`bGIb2=FRP4S^_HS6XuMG3x=dw@6o^A=5(&P<aFd$LMrML=8WlMy?HLB5&D(UkA@blB-N0L`UJX@)jB>j'
    'z)1~p+$IaLNsm5LC8VN0LGHCZkaHPbL&MvNJ^`fN;c7@leZrhE^^gq7w-yO9&>fUtYjo&WLMkd0=8U;<?wvPKFkjCx(l?5I`IUf*'
    '0);tavdI={GIhq0r`juU+Q7MnR1_%88dLb7x6Zzk+H&Sy-f?!v=ry3CK4H$7L}pW!4k2n7;e)+G0?<846`)SkC(Ibr9#ZgukR+NJ'
    '``t%pwtsUCsVGpehZ8%Q7|<uw-KVT?vWtf`jG{DQo|yWkQ1a)~_+neXCk^V;R6;6B6MRj!5z{j7nHm)Oy{BIc-8WVODk>9lTQO;*'
    'H7Skv;F)V<D^NA53Q|#-Fkei)&hFqwa``FYd!#_5)xH{1QJFAb%sn-kJtv<PefEkT#vbY|A{C_x^TlMEcky*{`vIP73p~f_;f@+o'
    'QJT<)4)Z}=X;q4C0MWf>Wpb~-E+Q4R2{Xnd_OZg3;{;!nKJP_uNClAeOhic5H?cK9u-0aLC#4zJ@^bv)EuBe~kczs5`C`(Bb5{qv'
    'lLCETe({YyQ8k>RHes%qq^}wC!l9<T?q7QJ0XNzpq=r<~Cd?HR(Iki5bLYtQ1QwuiTAU~$^|d&0+!d2>Y)(h7^=f<^`~Wt$dR{{c'
    'BPoq17zM2f{oNDzSmVU)E5o&+pHn5ID3Y>x0$}CqRW$P3w-FPTU|)VEpg4l^Vh7yIZ>4P$w@sZ*nr%`Qq$H9$Br;HD+TFL|e!-Y?'
    '9$eiqTSDptQs#?^u(HjEa>S)R!g-oe1*174Biq;^Jm`Z&il2?w8l?;Ly;nkNMp9<5y^e2{nzQHQ|2>g{NXlkJY5MeyWNTmVq`*Y|'
    'aHv~cN=U6p${HsIyOY5?DUJFb9~U4E(L0eUAr-|5bH!XN&*qf$ImLHIbhV6BL@J6C=89<!RXbg(cX;`hF!O~2?0}*KRMaK_%|h{j'
    'H%=r~DSa*Lu7a4*V^TGwqBvounEKMk2YTOTykj53*LF0i3Q|#=Fjq`j=#^f6w+GoEd@X@BR@_~eDj^lciRIM}@wTydyRkFJ;ycu#'
    '?{GD#3Q`ay3UkF|ix|?L*}We(MiZ6V$E5~TR4B|9)0!(e?ZniN<m@L#m!3SV0Tl%bGsV>JaVLW$o>&)?UXOeCQoR~bQJ*kROy0h{'
    '_cf9@M9*Rs=+XEZNKu+FJ517e+Z)1?=^Oi^JJfNoM+Is~MQOt9FlB4noAx4|HI|Q3nJ*HK)+S0wMQy?iF*V^8&LfGTZ_TUmHrLii'
    'ssvP&Cib1_n4C)!>&QJ)n?|3g8c<Q0Fgr}aO*7s5YAZy2M%2XFwuCDp6_p9I!xX*Qydw;ggm-zztlokwZQEQzYDHzj{4iyLn0ucj'
    'l@7kWNH$N}5qb%yC`_PtVeT!tFadlcbnC(-Ze^l`R8%I+4)bUh<>%$S4O7qC)h4#4$1Td1fQrh5%}}%HNO|W3!n0Wb^!Do(ml9A>'
    'oiH;@KFeLkW~L+Y5#KnvWuHD$C8VM{VP=?215W;wGJow`j~_&<>+vO|qB>z_n2PF}XD9K5{H?`hN(X)=prShAW|*QFEI{YvDaCZl'
    '@o;vB=ryFGJYi;-XYlX?^G=F{mXlg*cF$}PsVGmF8>WQ_JKv>@IIfEe>xt^di4syf$`fXXNg8E%PP>zG$D^W?e*A4aQ9~*U6np~;'
    '&VA6&f-j_oHM%sZ3Q|#^FjLH<9|`46vjlfNxu#q^p`TMFq@qG$rkMJVdmp_&rbx3Xa8A(n+Et*ULSd$uvTRH@RVZwa+q&xPSRGK*'
    'kctY0EPXczkFTkGcO26g{dPAGVQGP)1XL6#%oCI4@u+&2(P@2YNrKa(qBWqRKw*}c!cREyt|XyMxm9^Oef8CViu!~(Vh({@lz)rR'
    '!TvtpRg18t4aaIoMSbG9KB27gc1#+Ncj=t&{oJ6wIVg<s#PLDVxf4fB&7IC&qdfR<8-$cliqeERVw!*ANe8brI+t@>F_-ja@)}T4'
    'na~K1EjjXvQ&N#X1H1=NWAuHRR0*l5OqeC6b-7q$ge1nza+_R4^!Dq!uLRUr$^;L0#1wO1%<H^1PvE<X@N^?)5h#qHOqkGT)W~=9'
    'k~NIo&I#!aZ8e}Mf<o^`TJB5w9`=p<1!scolxk2Fpg4lkXaXl%8F$Qgzn!h0y8DM{d=9;)t%j6DQu?~}OpTe{eSrE0vp=M4QP^x!'
    'C8SOuH7D@`86UJ_+M&A(^~`RpK^1_a5kZ+Up|RY^<#Rl|ANjmbR1v5dL76zQfRovqN0xv^U52kg1QC%262Ra~u3u-aLGMGkOILuY'
    'b@LifD}vH@GdElpotaH^>|2V`z@wGE8d6b~P)0oLgZ;q9;ES#EkaaqX`d1_MU-kF??azPy^y~k!Kl%%_=fCLx`~2y%{g*%dY#2Wq'
    '$Y1{SgMRa#_6Pp_+vi_?)7SGKfA`0q|NP_ce);w1&p-e2C;Z)~U;q4%`p@@&{HNdj@zbwA{L`mjfBgOrKmVX#@#*s)|N6T>|LkyK'
    '*m|5P|5-8N9{=UL|MAzK|MHXnsXzbn{XhQcryu{-erNj~?Kk$H#=rWHp?<1g|Cs$heiT3b;qz~we)?X&=Ev{<_`j8&*>}OwyIpv$'
    'X)Z$g`TY9%#~;4`^!fYWKL0^quKmgXFwQ^tpZ)udzu)USOHv5`(-^-WLtn(_pMU+Izy0|AkAL~}-+ub>hrcI2{towuuty|4BIfZG'
    '<DoxaKYh~w_5aOj@elk@fBEmP?+CsVw5$JbzyAGOI{Ujpcx=Ck|H}W@JL2!AkSO8j*Z7VWe8*qE<oJFNzYE{XU%emxZlK(acm#W_'
    'qwnXpUw-(M{Q%MLaeYS%H8T9$573W#m45UcN52K0@A&6`{ty2L_vC6J'
)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def utc():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    def convert(value):
        if hasattr(value, "tolist"):
            return value.tolist()
        if isinstance(value, Path):
            return str(value)
        raise TypeError(type(value).__name__)
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False, default=convert) + "\n", encoding="utf-8")
    tmp.replace(path)


def validate_target(target, calibration):
    if not isinstance(target, dict) or not target or not set(target).issubset(JOINTS):
        raise ValueError("Target must contain one or more known joints")
    for name, value in target.items():
        value = finite_number(value, name)
        c = calibration[name]
        half = (c.range_max - c.range_min) * 180 / 4095
        low, high = (0., 100.) if name == "gripper" else (-half, half)
        if not low <= value <= high:
            raise ValueError(f"{name}: {value:.3f} outside live motor range {low:.3f}..{high:.3f}")


def clamp_target(target, calibration):
    """arm-002 override: clip requested commands, never measured positions."""
    if not isinstance(target, dict) or not target or not set(target).issubset(JOINTS):
        raise ValueError("Target must contain one or more known joints")
    clipped = {}
    for name, requested in target.items():
        value = finite_number(requested, name)
        c = calibration[name]
        half = (c.range_max - c.range_min) * 180 / 4095
        low, high = (0., 100.) if name == "gripper" else (-half, half)
        clipped[name] = min(high, max(low, value))
    validate_target(clipped, calibration)
    return clipped


def clamping_summary(targets, calibration):
    changed = {}
    for target in targets:
        clipped = clamp_target(target, calibration)
        for name, actual in clipped.items():
            requested = float(target[name])
            if actual != requested:
                item = changed.setdefault(name, {"targets": 0, "requested_min": requested,
                    "requested_max": requested, "commanded_min": actual, "commanded_max": actual})
                item["targets"] += 1
                item["requested_min"] = min(item["requested_min"], requested)
                item["requested_max"] = max(item["requested_max"], requested)
                item["commanded_min"] = min(item["commanded_min"], actual)
                item["commanded_max"] = max(item["commanded_max"], actual)
    return changed


@contextmanager
def connected_bus(port, joints=JOINTS):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    if port == "auto":
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        if len(ports) != 1:
            raise ValueError(f"Specify --port: expected one local serial device, found {[p.device for p in ports]}")
        port = ports[0].device
    if not joints or not set(joints).issubset(JOINTS):
        raise ValueError("Specify one or more known joints")
    bus = FeetechMotorsBus(port=port, motors={
        j: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if j == "gripper" else MotorNormMode.DEGREES)
        for i, j in enumerate(JOINTS) if j in joints})
    try:
        bus.connect()
        bus.calibration = bus.read_calibration()
        for j in joints:
            c = bus.calibration[j]
            if not 0 <= c.range_min < c.range_max <= 4095:
                raise ValueError(f"Invalid live motor calibration for {j}")
        yield bus
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def preflight(bus, targets):
    # Validate every target before the first goal/torque write.
    targets = list(targets)
    if not targets:
        raise ValueError("Specify at least one movement target")
    for target in targets:
        validate_target(clamp_target(target, bus.calibration), bus.calibration)
    joints = [j for j in JOINTS if any(j in target for target in targets)]
    for j in joints:
        if bus.read("Operating_Mode", j, normalize=False) != 0:
            raise ValueError(f"{j} must be in position mode")
        if bus.read("Phase", j, normalize=False) & 0x10:
            raise ValueError(f"{j} must use single-turn feedback")
    current = bus.sync_read("Present_Position", joints)
    validate_target(current, bus.calibration)
    return current


def enable_at_current_position(bus, joints=JOINTS):
    joints = list(joints)
    if not joints or not set(joints).issubset(JOINTS):
        raise ValueError("Specify one or more known joints")
    raw = bus.sync_read("Present_Position", joints, normalize=False)
    for j, value in raw.items():
        c = bus.calibration[j]
        if not c.range_min <= value <= c.range_max:
            raise ValueError(f"{j} moved outside its calibrated range")
    torque = bus.sync_read("Torque_Enable", joints, normalize=False)
    bus.sync_write("Goal_Position", raw, normalize=False)
    disabled = [j for j in joints if not torque[j]]
    if disabled:
        bus.enable_torque(disabled)


def move(bus, target, seconds):
    target = clamp_target(target, bus.calibration)
    start = bus.sync_read("Present_Position", list(target))
    validate_target(start, bus.calibration)
    started = time.monotonic()
    while True:
        fraction = min((time.monotonic() - started) / seconds, 1.)
        alpha = fraction * fraction * (3 - 2 * fraction)
        command = dict(target) if fraction == 1 else {j: start[j] + alpha * (target[j] - start[j]) for j in target}
        bus.sync_write("Goal_Position", command, normalize=True)
        if fraction == 1:
            break
        time.sleep(.02)


def settle(bus, target, seconds):
    """Record positioning accuracy; image quality decides camera calibration."""
    target = clamp_target(target, bus.calibration)
    time.sleep(seconds)
    measured = bus.sync_read("Present_Position", list(target))
    # Device faults and communication failures still stop the routine.
    for j in target:
        status = bus.read("Status", j, normalize=False)
        if status:
            raise RuntimeError(f"{j}: motor fault {status} while settling")
    errors = {j: abs(finite_number(measured[j], j) - target[j])
              for j in target if j != "gripper"}
    missed = {j: error for j, error in errors.items() if error > 4}
    if missed:
        print("Position warning (degrees from target): " + json.dumps(missed, sort_keys=True)
              + ". Photo detection and calibration quality checks remain required.", flush=True)
    return {"measured": measured, "error_degrees": errors, "within_tolerance": not missed}

def detect_board(image, pattern, thorough=False):
    import cv2
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    if thorough:
        flags |= cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    return corners.reshape(-1, 2) if ok else None


def wait_for_board(reader, pattern, camera="top"):
    import numpy as np
    print("At pose 1, gripper open. Move the checkerboard into the jaws, then hold it steady.", flush=True)
    previous, count = None, 0
    baseline, initialized, placement_changed = None, False, False
    last_notice = time.monotonic()
    while True:
        try:
            image, _ = reader.read(camera)
        except RuntimeError:
            count, previous = 0, None
            time.sleep(.2)
            continue
        corners = detect_board(image, pattern)
        if not initialized:
            baseline = corners
            initialized = True
        elif corners is not None and baseline is None:
            placement_changed = True
        elif corners is not None and baseline is not None and min(float(np.sqrt(np.mean((corners - b) ** 2))) for b in (baseline, baseline[::-1])) > 12:
            placement_changed = True
        if corners is None:
            count, previous = 0, None
        else:
            change = min(np.max(np.linalg.norm(corners - p, axis=1)) for p in (previous, previous[::-1])) if previous is not None else math.inf
            count = count + 1 if change < 2 else 1
            previous = corners
            # A board left on the tabletop from the last attempt must not
            # immediately trigger another grasp of empty air.
            if count >= 5 and placement_changed:
                print("Checkerboard detected. Closing the gripper.", flush=True)
                return
        if time.monotonic() - last_notice > 10:
            print(f"Waiting for a steady {pattern[0]} x {pattern[1]} inner-corner board in the {camera} view...", flush=True)
            last_notice = time.monotonic()


def capture_pose(bus, reader, output, name, pattern, cameras, timeout):
    """Save a stationary top-camera photo and its measured joint angles."""
    import cv2
    deadline, best = time.monotonic() + timeout, {}
    while time.monotonic() < deadline:
        before = bus.sync_read("Present_Position", list(JOINTS))
        after_time = time.monotonic()
        frames = {cam: reader.read(cam, after=after_time) for cam in cameras}
        after = bus.sync_read("Present_Position", list(JOINTS))
        if max(abs(before[j] - after[j]) for j in ARM_JOINTS) > .5:
            continue
        for camera, (image, meta) in frames.items():
            corners = detect_board(image, pattern)
            if camera not in best or (best[camera]["corners"] is None and corners is not None):
                best[camera] = {"image": image, "corners": corners, "frame": meta,
                                "joints": {j: (before[j] + after[j]) / 2 for j in JOINTS}}
        if all(best[c]["corners"] is not None for c in cameras):
            break
    if len(best) != len(cameras):
        raise RuntimeError(f"{name}: no stable camera/joint snapshot")
    metadata = {"captured_at_utc": utc(), "cameras": {}}
    (output / "captures").mkdir(exist_ok=True)
    for camera, entry in best.items():
        # Retry difficult, tilted grids only while stationary. Keep the live
        # detector fast during motion and while waiting for the board.
        thorough = entry["corners"] is None
        if thorough:
            entry["corners"] = detect_board(entry["image"], pattern, thorough=True)
        file = output / "captures" / f"{name}_{camera}.png"
        if not cv2.imwrite(str(file), entry["image"]):
            raise OSError(f"Could not save {file}")
        corners = entry["corners"]
        metadata["cameras"][camera] = {"image": str(file.relative_to(output)), "frame": entry["frame"],
                                      "joints": entry["joints"], "image_size": list(entry["image"].shape[1::-1]),
                                      "detection": "exhaustive_accuracy" if thorough else "normal",
                                      "corners": corners.tolist() if corners is not None else None}
        print(f"{name}, {camera}: photo saved; board {'detected' if corners is not None else 'not detected'}", flush=True)
    write_json(output / "captures" / f"{name}.json", metadata)
    return metadata


def object_points(pattern, square_m):
    import numpy as np
    points = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    points[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_m
    return points


def fit_intrinsics(views, pattern, square_m):
    """Estimate from this run's image corners, with no input camera matrix."""
    import cv2
    import numpy as np
    usable = []
    for view in views:
        if view["corners"] is None:
            continue
        a = np.asarray(view["corners"])
        if any(min(float(np.sqrt(np.mean((a - np.asarray(v["corners"])) ** 2))),
                   float(np.sqrt(np.mean((a - np.asarray(v["corners"])[::-1]) ** 2)))) < 3 for v in usable):
            continue
        usable.append(view)
    if len(usable) < 3:
        raise ValueError(f"Only {len(usable)} distinct checkerboard views; at least 3 relative camera/board views are needed")
    size = tuple(usable[0]["image_size"])
    if any(tuple(v["image_size"]) != size for v in usable):
        raise ValueError("Image resolution changed during capture")
    points = object_points(pattern, square_m)
    corners = [np.asarray(v["corners"], np.float32).reshape(-1, 1, 2) for v in usable]
    if any(c.shape != (len(points), 1, 2) or not np.isfinite(c).all() for c in corners):
        raise ValueError("Invalid checkerboard corner observations")
    diversity = max(min(float(np.sqrt(np.mean((a - b) ** 2))), float(np.sqrt(np.mean((a - b[::-1]) ** 2))))
                    for i, a in enumerate(corners) for b in corners[i + 1:])
    if diversity < 3:
        raise ValueError("The checkerboard stayed in the same position relative to this camera; these photos cannot determine its intrinsics")
    # Fit the full distortion model when autonomous extra photos provide enough
    # views. With fewer views, use the explicitly labelled radial-only model.
    flags = 0 if len(corners) >= 10 else cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        [points for _ in corners], corners, size, None, None, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    if not np.isfinite(K).all() or not np.isfinite(D).all() or not math.isfinite(rms):
        raise ValueError("Calibration produced non-finite coefficients")
    if not (.1 * size[0] < K[0, 0] < 10 * size[0] and .1 * size[1] < K[1, 1] < 10 * size[1]
            and 0 <= K[0, 2] < size[0] and 0 <= K[1, 2] < size[1]):
        raise ValueError("Fitted focal length/principal point is implausible; board views need more tilt")
    normals = [cv2.Rodrigues(r)[0][:, 2] for r in rvecs]
    tilt_span = max(math.degrees(math.acos(np.clip(abs(float(a @ b)), 0, 1)))
                    for i, a in enumerate(normals) for b in normals[i + 1:])
    if tilt_span < 2:
        raise ValueError("Checkerboard orientations are effectively parallel; more relative tilt is needed to determine intrinsics")
    per_view = []
    for obs, r, t in zip(corners, rvecs, tvecs):
        projected, _ = cv2.projectPoints(points, r, t, K, D)
        per_view.append(float(np.sqrt(np.mean(np.sum((obs - projected) ** 2, axis=2)))))
    if max(per_view) > 3:
        raise ValueError(f"Reprojection error exceeds 3 pixels: {per_view}")
    return {"status": "estimated", "model": "opencv_pinhole", "camera_matrix": K.tolist(),
            "dist_coeffs": D.ravel().tolist(), "dist_coeff_order": ["k1", "k2", "p1", "p2", "k3"],
            "fitted_distortion_terms": ["k1", "k2", "p1", "p2", "k3"] if not flags else ["k1", "k2"],
            "fixed_zero_terms": [] if not flags else ["p1", "p2", "k3"],
            "image_size": list(size), "rms_px": float(rms), "per_view_rms_px": per_view,
            "board_normal_span_deg": tilt_span, "views": len(usable),
            "board": {"inner_corners": list(pattern), "square_m": square_m},
            "validation": "Estimated from this run's photos; reprojection error is not independent accuracy validation."}


def load_trajectory(path, cfg):
    data = read_json(path)
    if (data.get("schema_version") != 1 or data.get("type") != "joint_trajectory" or
            data.get("coordinate_space") != "lerobot_calibrated" or data.get("units") != UNITS or
            data.get("summary", {}).get("state") != "complete"):
        raise ValueError("Expected a completed, calibrated joint_trajectory file")
    samples = data["samples"]
    if len(samples) < 2:
        raise ValueError("Trajectory needs at least two samples")
    times = np.array([finite_number(s["time_s"], "time_s") for s in samples])
    if times[0] < 0 or np.any(np.diff(times) <= 0):
        raise ValueError("Trajectory times must be nonnegative and strictly increasing")
    values = []
    for sample in samples:
        if set(sample["joints"]) != set(JOINTS):
            raise ValueError("Every trajectory sample must include all five arm joints and gripper")
        values.append([finite_number(sample["joints"][j], j) for j in JOINTS])
    gap = float(np.max(np.diff(times)))
    if gap > cfg["quality"]["max_recording_gap_seconds"]:
        raise ValueError(f"Recording has a {gap:.3f}s gap, exceeding the configured maximum")
    values = np.asarray(values)
    grip = JOINTS.index("gripper")
    recorded_closed, released = float(values[0, grip]), float(values[-1, grip])
    if (np.any((values[:, grip] < 0) | (values[:, grip] > 100)) or
            not 0 <= recorded_closed < released <= 100 or not 0 <= CLOSED_GRIPPER < released):
        raise ValueError("Placement recording must start holding the board and end with the gripper open")
    # The recorded hold was ~7%, which would loosen the new 0% grip during the
    # lead-in and early replay. Rebase only the gripper's opening progression:
    # keep the new hold until recorded release, retain the final opening, and
    # preserve every arm sample and timestamp. The recording on disk is intact.
    opening = (values[:, grip] - recorded_closed) / (released - recorded_closed)
    values[:, grip] = CLOSED_GRIPPER + np.maximum(opening, 0) * (released - CLOSED_GRIPPER)
    return times - times[0], values, {"samples": len(samples), "duration_seconds": float(times[-1] - times[0]),
                                               "largest_recording_gap_seconds": gap, "sha256": sha256(path),
                                               "gripper_override": {"recorded_hold_percent": recorded_closed,
                                                                    "closed_target_percent": CLOSED_GRIPPER,
                                                                    "release_open_percent": released,
                                                                    "method": "Rebase opening progression; retain release timing"},
                                               "replay_duration_seconds": active_trajectory_end(times - times[0], values),
                                               "timing": "Recorded relative timestamps and initial still time retained; final stationary tail skipped; exact endpoint retained"}


def trajectory_target(times, values, elapsed):
    return dict(zip(JOINTS, [float(np.interp(elapsed, times, values[:, i])) for i in range(len(JOINTS))]))


def active_trajectory_end(times, values):
    """Remove only the final stationary tail; preserve the path and endpoint."""
    tolerance = np.array([.25] * len(ARM_JOINTS) + [1.])
    moving = np.flatnonzero(np.any(np.abs(values - values[-1]) > tolerance, axis=1))
    index = min(int(moving[-1]) + 1, len(times) - 1) if len(moving) else 0
    return float(times[index])


def replay(bus, times, values, cfg, on_target=None):
    start = time.monotonic()
    end = active_trajectory_end(times, values)
    while True:
        elapsed = (time.monotonic() - start) * cfg["replay_speed"]
        target = clamp_target(trajectory_target(times, values, times[-1] if elapsed >= end else elapsed), bus.calibration)
        bus.sync_write("Goal_Position", target, normalize=True)
        if on_target is not None:
            on_target(elapsed, target)
        if elapsed >= end:
            return target
        time.sleep(1 / cfg["command_hz"])


class FrameReader:
    """Read fresh frames from the existing camera service without reopening USB."""
    def __init__(self, directory):
        self.directory, self.last = Path(directory), {}

    def read(self, camera, after=-math.inf, timeout=5):
        deadline, error = time.monotonic() + timeout, "no frame"
        while time.monotonic() < deadline:
            try:
                data = (self.directory / f"{camera}.frame").read_bytes()
                if len(data) < 8 or data[:4] != b"CMD1":
                    raise ValueError("bad camera frame header")
                length = struct.unpack("<I", data[4:8])[0]
                meta = json.loads(data[8:8 + length])
                age = time.monotonic() - meta["t_mono"]
                if meta["cam"] != camera or not 0 <= age <= .5:
                    raise ValueError(f"stale/wrong frame (age {age:.2f}s)")
                if meta["t_mono"] <= after or meta["seq"] == self.last.get(camera):
                    time.sleep(.02)
                    continue
                image = cv2.imdecode(np.frombuffer(data[8 + length:], np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("could not decode JPEG")
                self.last[camera] = meta["seq"]
                return image, meta
            except (OSError, ValueError, KeyError, struct.error) as exc:
                error = str(exc)
                time.sleep(.05)
        raise RuntimeError(f"{camera}: {error}; check the camera service")


@contextmanager
def camera_source(args):
    """Use a fresh shared feed, or own the USB camera without another service."""
    try:
        FrameReader(args.frames_directory).read("top", timeout=.6)
        shared = True
    except RuntimeError:
        shared = False
    if shared:
        yield
        return
    capture = cv2.VideoCapture(CAMERAS["top"], cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("Top USB camera could not be opened and no fresh shared feed is available")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_FPS, 30)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    stop = threading.Event()
    original = args.frames_directory
    with tempfile.TemporaryDirectory(prefix="armfarm_camera_") as directory:
        args.frames_directory = directory
        def collect():
            seq = 0
            while not stop.is_set():
                ok, image = capture.read()
                captured = time.monotonic()
                if not ok:
                    stop.wait(.1)
                    continue
                ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    continue
                seq += 1
                meta = json.dumps({"cam": "top", "seq": seq, "t_mono": captured}).encode()
                temporary = Path(directory) / "top.tmp"
                temporary.write_bytes(b"CMD1" + struct.pack("<I", len(meta)) + meta + jpeg.tobytes())
                temporary.replace(Path(directory) / "top.frame")
        thread = threading.Thread(target=collect, daemon=True)
        thread.start()
        try:
            FrameReader(directory).read("top", timeout=5.)
            yield
        finally:
            stop.set()
            thread.join(timeout=2.)
            capture.release()
            thread.join(timeout=2.)
            args.frames_directory = original


class ReleasedBoardPhoto:
    """Take the table photo during release/retraction, never after the final hold."""
    def __init__(self, args, output, release_percent):
        self.args, self.output, self.release_percent = args, output, release_percent
        self.trigger, self.stop = threading.Event(), threading.Event()
        self.released_at, self.release_path_s = None, None
        self.result, self.error = None, None
        self.thread = threading.Thread(target=self.capture, daemon=True)

    def observe_target(self, elapsed, target):
        # This callback does no image processing or bus reads; servo streaming
        # continues at its normal rate while the camera thread saves the photo.
        if self.released_at is None and target["gripper"] >= self.release_percent:
            self.released_at, self.release_path_s = time.monotonic(), elapsed
            self.trigger.set()

    def capture(self):
        try:
            self.trigger.wait()
            if self.stop.is_set():
                return
            reader = FrameReader(self.args.frames_directory)
            deadline = self.released_at + self.args.capture_timeout
            previous = None
            while not self.stop.is_set() and time.monotonic() < deadline:
                image, frame = reader.read("top", after=self.released_at, timeout=1.)
                corners = detect_board(image, tuple(self.args.pattern))
                if corners is None:
                    previous = None
                    continue
                change = min(float(np.max(np.linalg.norm(corners - p, axis=1))) for p in (previous, previous[::-1])) if previous is not None else math.inf
                previous = corners
                if change > 2:
                    continue
                path = self.output / "captures" / "workspace_top.png"
                if not cv2.imwrite(str(path), image):
                    raise OSError(f"Could not save {path}")
                latency = frame["t_mono"] - self.released_at
                self.result = {"captured_at_utc": utc(), "capture_phase": "after_release_during_retraction",
                               "release_path_seconds": self.release_path_s, "seconds_after_open_command": latency,
                               "cameras": {"top": {"image": str(path.relative_to(self.output)), "frame": frame,
                                   "image_size": list(image.shape[1::-1]), "corners": corners.tolist()}}}
                write_json(self.output / "captures" / "workspace.json", self.result)
                print(f"Workspace photo saved {latency:.2f}s after the release command. You may remove the board; processing uses this saved photo.", flush=True)
                return
            if not self.stop.is_set():
                raise CalibrationRetry("No steady full checkerboard view immediately after release")
        except Exception as exc:
            self.error = exc

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.trigger.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            raise RuntimeError("Workspace camera thread did not stop")


def replay_and_capture_workspace(bus, times, values, cfg, args, output):
    release_percent = float(values[-1, JOINTS.index("gripper")]) * .95
    with ReleasedBoardPhoto(args, output, release_percent) as photo:
        final = replay(bus, times, values, cfg, on_target=photo.observe_target)
        # Usually already finished, since the photo is taken during retraction.
        photo.thread.join(timeout=args.capture_timeout + 1.)
        if photo.error is not None:
            raise CalibrationRetry(f"Workspace capture failed: {photo.error}") from photo.error
        if photo.result is None:
            raise CalibrationRetry("Workspace photo was not captured after release")
        return final, photo.result


def camera_controls(device):
    result = subprocess.run(["v4l2-ctl", "-d", device, "--list-ctrls"], capture_output=True, text=True, check=True, timeout=5)
    controls = {}
    for line in result.stdout.splitlines():
        match = re.match(r"\s*(\w+)\s+0x.*?value=(-?\d+)", line)
        if match and any(word in match[1] for word in ("focus", "zoom")):
            controls[match[1]] = int(match[2])
    return controls


def freeze_focus(cameras):
    result = {}
    for camera in cameras:
        try:
            before = camera_controls(CAMERAS[camera])
            for name, value in before.items():
                if "focus" in name and "auto" in name and value:
                    subprocess.run(["v4l2-ctl", "-d", CAMERAS[camera], "--set-ctrl", f"{name}=0"],
                                   capture_output=True, check=True, timeout=5)
            after = camera_controls(CAMERAS[camera])
            if any(v for name, v in after.items() if "focus" in name and "auto" in name):
                raise ValueError("autofocus did not turn off")
            result[camera] = {"before": before, "during_capture": after}
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            result[camera] = {"error": str(exc)}
            print(f"{camera}: focus lock unavailable; marked in report.", flush=True)
    return result


class ExtraViews:
    """Collect extra top-camera views without commanding or reading motors."""
    def __init__(self, args, output):
        self.args, self.output = args, output
        self.views = {cam: [] for cam in args.cameras}
        self.errors, self.stop = [], threading.Event()
        self.thread = threading.Thread(target=self.collect, daemon=True)

    def collect(self):
        reader = FrameReader(self.args.frames_directory)
        try:
            while not self.stop.is_set():
                for cam in self.args.cameras:
                    if self.stop.is_set() or len(self.views[cam]) >= 20:
                        continue
                    try:
                        image, meta = reader.read(cam, timeout=1)
                        corners = detect_board(image, tuple(self.args.pattern))
                        if corners is None:
                            continue
                        if any(min(np.sqrt(np.mean((corners - np.array(v["corners"])) ** 2)),
                                   np.sqrt(np.mean((corners - np.array(v["corners"])[::-1]) ** 2))) < 12 for v in self.views[cam]):
                            continue
                        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                        x, y, w, h = cv2.boundingRect(corners.astype(np.float32))
                        roi = gray[max(0, y):min(y + h, gray.shape[0]), max(0, x):min(x + w, gray.shape[1])]
                        if roi.size == 0 or cv2.Laplacian(roi, cv2.CV_64F).var() < 50:
                            continue
                        path = self.output / "captures" / f"extra_{cam}_{len(self.views[cam]) + 1:02d}.jpg"
                        if not cv2.imwrite(str(path), image):
                            raise OSError(f"Cannot write {path}")
                        self.views[cam].append({"image": str(path.relative_to(self.output)), "corners": corners.tolist(),
                                                "image_size": list(image.shape[1::-1]), "frame": meta})
                    except (RuntimeError, OSError) as exc:
                        if len(self.errors) < 10:
                            self.errors.append(str(exc))
                self.stop.wait(.3)
        except Exception as exc:
            self.errors.append(str(exc))

    def __enter__(self):
        (self.output / "captures").mkdir(exist_ok=True)
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            raise RuntimeError("Camera sampler did not stop")
        write_json(self.output / "captures" / "extra_views.json", {"cameras": self.views, "errors": self.errors})


def map_config(args):
    return {"replay_speed": 1., "command_hz": 50.,
            "quality": {"max_recording_gap_seconds": 1., "max_reprojection_px": 3.}}


def build_table_map(placed, profile, args, output):
    """Map raw camera pixels to metres on the checkerboard's table plane."""
    require_board(placed, args, "workspace")
    view = placed["cameras"]["top"]
    width, height = view["image_size"]
    if profile["image_size"] != [width, height]:
        raise ValueError("Map and lens calibration resolutions differ")
    matrix = np.asarray(profile["camera_matrix"], dtype=float)
    distortion = np.asarray(profile["dist_coeffs"], dtype=float)
    corners = np.asarray(view["corners"], dtype=float)
    undistorted = cv2.undistortPoints(corners.reshape(-1, 1, 2), matrix, distortion, P=matrix)
    xy = object_points(tuple(args.pattern), args.square_mm / 1000)[:, :2]
    homography, _ = cv2.findHomography(undistorted, xy, 0)
    if homography is None or not np.isfinite(homography).all():
        raise ValueError("Checkerboard does not define a table plane")
    predicted = cv2.perspectiveTransform(xy.astype(float).reshape(-1, 1, 2), np.linalg.inv(homography))
    residual = np.linalg.norm(predicted - undistorted, axis=2)
    rms = float(np.sqrt(np.mean(residual ** 2)))
    if rms > map_config(args)["quality"]["max_reprojection_px"]:
        raise ValueError(f"Table-plane reprojection error {rms:.3f} px exceeds 3 px")
    pixels = np.indices((height, width), dtype=np.float32)[::-1].transpose(1, 2, 0).reshape(-1, 1, 2)
    corrected = cv2.undistortPoints(pixels, matrix, distortion, P=matrix)
    table = cv2.perspectiveTransform(corrected, homography).reshape(height, width, 2)
    denominator = corrected.reshape(-1, 2) @ homography[2, :2] + homography[2, 2]
    center_denom = undistorted.reshape(-1, 2).mean(axis=0) @ homography[2, :2] + homography[2, 2]
    valid = (np.isfinite(table).all(axis=2) & (np.abs(denominator.reshape(height, width)) > 1e-8)
             & ((denominator * center_denom).reshape(height, width) > 0))
    table[~valid] = np.nan
    observed = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(observed, cv2.convexHull(corners.astype(np.float32)).astype(np.int32), 1)
    np.savez_compressed(output / "workspace_map.npz", xy_table_m=table, valid=valid,
                        observed_board_region=observed.astype(bool))
    image = cv2.imread(str(output / view["image"]))
    if image is None or list(image.shape[1::-1]) != [width, height]:
        raise ValueError("Saved table image is missing or has a different resolution")
    if not cv2.imwrite(str(output / "workspace_undistorted.png"), cv2.undistort(image, matrix, distortion)):
        raise OSError("Could not save distortion-corrected reference image")
    mapped = {"schema_version": 2, "type": "table_plane_2d", "coordinate_frame": "checkerboard_table",
              "units": "metres", "image_size": [width, height], "camera": "top",
              "origin": "first detected inner corner in saved workspace image",
              "axes": {"x": "checkerboard columns", "y": "checkerboard rows"},
              "board": {"inner_corners": args.pattern, "square_m": args.square_mm / 1000},
              "robot_alignment": "not_calibrated", "height": "not_measured",
              "homography_undistorted_pixel_to_table_m": homography,
              "camera_matrix": matrix, "dist_coeffs": distortion,
              "map": "workspace_map.npz", "pixel_space": "raw", "array_indexing": "[y, x]",
              "source_image": view["image"], "undistorted_reference": "workspace_undistorted.png",
              "reprojection_rms_px": rms, "reprojection_max_px": float(residual.max())}
    write_json(output / "workspace_map.json", mapped)
    return mapped


def reuse_attempt(source, args, output):
    """Recover a saved capture set without repeating checkerboard movements."""
    source = source.resolve()
    saved = read_json(source / "intrinsics" / "top.json")
    if saved.get("device") != CAMERAS["top"]:
        raise ValueError("Saved calibration belongs to a different top camera")
    views, checked = [], {}
    for name in ("pose_1", "pose_2", "pose_3", "workspace"):
        capture = read_json(source / "captures" / f"{name}.json")
        checked[name] = require_board(capture, args, name)
        view = capture["cameras"]["top"]
        image = cv2.imread(str(source / view["image"]))
        if image is None or list(image.shape[1::-1]) != view["image_size"]:
            raise ValueError(f"Saved {name} image is missing or invalid")
        views.append(view)
    extra = source / "captures" / "extra_views.json"
    if extra.exists():
        views.extend(read_json(extra)["cameras"]["top"])
    profile = fit_intrinsics(views, tuple(args.pattern), args.square_mm / 1000)
    profile.update(camera="top", device=CAMERAS["top"], calibrated_at_utc=utc(),
                   optical_controls=saved.get("optical_controls", {}), reused_from=str(source))
    controls = saved.get("optical_controls", {}).get("during_capture")
    if controls is not None and camera_controls(CAMERAS["top"]) != controls:
        raise ValueError("Camera focus/zoom differs from the saved calibration")
    shutil.copytree(source / "captures", output / "captures")
    write_json(output / "intrinsics" / "top.json", profile)
    write_json(output / "intrinsics.json", {"schema_version": 1, "created_at_utc": utc(), "cameras": {"top": profile}})
    mapped = build_table_map(capture, profile, args, output)
    return {"state": "mapped", "reused_from": str(source), "photos": checked,
            "cameras": {"top": profile}, "map_reprojection_rms_px": mapped["reprojection_rms_px"]}


def load_home(path):
    data = read_json(path)
    if data.get("coordinate_space") != "lerobot_calibrated" or data.get("units") != {"arm": "degrees"}:
        raise ValueError("Home must contain calibrated arm degrees")
    joints = data.get("joints")
    if not isinstance(joints, dict) or set(joints) != set(HOME_JOINTS):
        raise ValueError("Home must constrain shoulder_pan, shoulder_lift, elbow_flex and wrist_flex only; wrist_roll and gripper are task dependent")
    return {j: finite_number(joints[j], j) for j in HOME_JOINTS}


def finish_at_home(bus, args, output, report, home):
    report.update(state="returning_home", calibration_accepted=True,
                  workspace_map="workspace_map.npz", coordinate_frame="checkerboard_table",
                  robot_alignment="not_calibrated", height="not_measured")
    write_json(output / "report.json", report)
    print("2D table map accepted. Returning to the saved home pose.", flush=True)
    move(bus, home, args.duration)
    settled = settle(bus, home, 2)
    report.update(state="complete", finished_at_utc=utc(),
                  home_reached=settled["within_tolerance"], home_joints=list(home),
                  home_error_degrees=settled["error_degrees"], final_joints=settled["measured"])
    if not settled["within_tolerance"]:
        report.setdefault("warnings", []).append({
            "code": "home_position_missed", "error_degrees": settled["error_degrees"],
            "message": "Camera calibration accepted; home position missed the 4-degree tolerance."})
    write_json(output / "report.json", report)
    print(f"Done. Camera intrinsics and 2D table map accepted: {output}", flush=True)
    return 0

@contextmanager
def process_lock():
    import fcntl
    with open("/tmp/armfarm-workspace-calibration.lock", "a") as file:
        try:
            fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Another calibration process is running") from None
        yield


class CalibrationRetry(ValueError):
    """An unusable capture/fit requires a new complete calibration attempt."""


def require_board(capture, args, name):
    view = capture["cameras"]["top"]
    corners = np.asarray(view.get("corners"), dtype=float)
    expected = args.pattern[0] * args.pattern[1]
    width, height = view["image_size"]
    if (corners.shape != (expected, 2) or not np.isfinite(corners).all()
            or np.any(corners < 0) or np.any(corners[:, 0] >= width)
            or np.any(corners[:, 1] >= height)):
        raise CalibrationRetry(f"{name}: full {args.pattern[0]} x {args.pattern[1]} checkerboard is not in view")
    return {"detected_corners": expected, "image": view.get("image")}


def return_to_pose_one(bus, args, retry=False):
    opened = {**POSES[0], "gripper": args.gripper_open}
    if retry:
        # Retain the current grip during the return; open only at pose 1.
        current = bus.sync_read("Present_Position", list(JOINTS))
        returning = {**POSES[0], "gripper": current["gripper"]}
        move(bus, returning, args.duration)
        settle(bus, returning, 2)
        move(bus, opened, 1.5)
    else:
        move(bus, opened, args.duration)
    settle(bus, opened, 2)


def run_attempt(bus, reader, args, output, times, values, report):
    """One complete top-camera attempt; never accept a missing held-board photo."""
    cfg = map_config(args)
    wait_for_board(reader, tuple(args.pattern), "top")
    move(bus, POSES[0], 1.5)
    settle(bus, POSES[0], 2)
    optics = freeze_focus(["top"])
    captures, checked = [], {}
    with ExtraViews(args, output) as extras:
        for index, pose in enumerate(POSES, 1):
            if index > 1:
                print(f"Moving to calibration pose {index}/3.", flush=True)
                move(bus, pose, args.duration)
                settle(bus, pose, 2)
            name = f"pose_{index}"
            try:
                capture = capture_pose(bus, reader, output, name, tuple(args.pattern), ["top"], args.capture_timeout)
            except RuntimeError as exc:
                raise CalibrationRetry(str(exc)) from exc
            # Fail immediately: do not continue to a later pose or release path.
            checked[name] = require_board(capture, args, name)
            captures.append(capture)
            report.update(state=f"captured_{name}", photos=checked)
            write_json(output / "report.json", report)
        print("All three top-camera photos contain the full checkerboard.", flush=True)
        print("Moving to the recorded path's start, then replaying checkerboard placement.", flush=True)
        first = dict(zip(JOINTS, values[0]))
        move(bus, first, args.duration)
        settle(bus, first, 2)
        final, placed = replay_and_capture_workspace(bus, times, values, cfg, args, output)
        settle(bus, final, 2)
        require_board(placed, args, "workspace")
    print("Calculating top-camera intrinsics from the new photos...", flush=True)
    try:
        views = [c["cameras"]["top"] for c in [*captures, placed]] + extras.views["top"]
        profile = fit_intrinsics(views, tuple(args.pattern), args.square_mm / 1000)
        profile.update(camera="top", device=CAMERAS["top"], calibrated_at_utc=utc(), optical_controls=optics["top"])
        try:
            after = camera_controls(CAMERAS["top"])
        except (OSError, subprocess.SubprocessError) as exc:
            if "during_capture" in optics["top"]:
                raise CalibrationRetry(f"Could not verify camera focus: {exc}") from exc
            after = {}
        if "during_capture" in optics["top"] and after != optics["top"]["during_capture"]:
            raise ValueError("Focus/zoom changed between calibration photos")
        if "error" in optics["top"]:
            profile["optics_warning"] = optics["top"]["error"]
        write_json(output / "intrinsics" / "top.json", profile)
        write_json(output / "intrinsics.json", {"schema_version": 1, "created_at_utc": utc(), "cameras": {"top": profile}})
        report["cameras"] = {"top": profile}
        print(f"top: {profile['views']} views, RMS {profile['rms_px']:.3f} px.", flush=True)
        print("Calculating the 2D table map in checkerboard coordinates...", flush=True)
        mapped = build_table_map(placed, profile, args, output)
    except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
        raise CalibrationRetry(str(exc)) from exc
    report.update(state="mapped", photos=checked,
                  workspace_map="workspace_map.npz", map_reprojection_rms_px=mapped["reprojection_rms_px"],
                  extra_view_errors=extras.errors, final_joints=bus.sync_read("Present_Position", list(JOINTS)))
    write_json(output / "report.json", report)
    return report


def run_local(args, output):
    cfg = map_config(args)
    home = load_home(args.home)
    times, values, trace = load_trajectory(args.trajectory, cfg)
    report = {"state": "checking", "started_at_utc": utc(), "mode": "top_camera_intrinsics_and_2d_table_map", "trajectory": trace}
    write_json(output / "report.json", report)
    reader = FrameReader(args.frames_directory)
    opened = {**POSES[0], "gripper": args.gripper_open}
    try:
        live_image, _ = reader.read("top")
        with connected_bus(args.port, joints=HOME_JOINTS if args.reuse_attempt else JOINTS) as bus:
            targets = [home] if args.reuse_attempt else [home, opened, *POSES, *[dict(zip(JOINTS, v)) for v in values]]
            report["target_clamping"] = clamping_summary(targets, bus.calibration)
            if report["target_clamping"]:
                print("arm-002: targets clipped to live calibrated limits: " + json.dumps(report["target_clamping"], sort_keys=True), flush=True)
            report["initial_joints"] = preflight(bus, targets)
            write_json(output / "report.json", report)
            if args.check:
                report.update(state="check_passed", motor_writes=False,
                              torque_enabled=bus.sync_read("Torque_Enable", list(bus.motors), normalize=False))
                write_json(output / "report.json", report)
                print("Check passed: top-camera stream, arm and required movement targets. No motor or focus writes.", flush=True)
                return 0
            if args.reuse_attempt:
                saved = read_json(args.reuse_attempt / "intrinsics" / "top.json")
                if saved["image_size"] != list(live_image.shape[1::-1]):
                    raise ValueError("Live top-camera resolution differs from saved calibration")
                report.update(reuse_attempt(args.reuse_attempt, args, output))
                enable_at_current_position(bus, joints=home)
                return finish_at_home(bus, args, output, report, home)
            enable_at_current_position(bus)
            attempt = 0
            while True:
                attempt += 1
                folder = output / "attempts" / f"{attempt:04d}"
                folder.mkdir(parents=True)
                report.update(state="returning_to_pose_1", attempt=attempt,
                              active_attempt=str(folder.relative_to(output)))
                write_json(output / "report.json", report)
                print(f"Attempt {attempt}: returning to pose 1 and opening the gripper.", flush=True)
                return_to_pose_one(bus, args, retry=attempt > 1)
                report["state"] = "waiting_for_checkerboard"
                write_json(output / "report.json", report)
                current = {"attempt": attempt, "started_at_utc": utc(), "state": "waiting_for_checkerboard"}
                try:
                    result = run_attempt(bus, reader, args, folder, times, values, current)
                except CalibrationRetry as exc:
                    current.update(state="retry", error=str(exc), finished_at_utc=utc())
                    write_json(folder / "report.json", current)
                    report.update(state="retrying", last_failure=str(exc))
                    write_json(output / "report.json", report)
                    print(f"Attempt {attempt} rejected: {exc}. Returning to pose 1 with an open gripper to try again.", flush=True)
                    continue
                # Publish only the accepted attempt. Failed captures remain in
                # their own directories and cannot masquerade as valid outputs.
                for name in ("captures", "intrinsics"):
                    shutil.copytree(folder / name, output / name, dirs_exist_ok=True)
                for name in ("intrinsics.json", "workspace_map.json", "workspace_map.npz", "workspace_undistorted.png"):
                    shutil.copyfile(folder / name, output / name)
                report.update(result)
                report.pop("last_failure", None)
                report["successful_attempt"] = str(folder.relative_to(output))
                return finish_at_home(bus, args, output, report, home)
    except BaseException as exc:
        report.update(state="stopped" if isinstance(exc, (KeyboardInterrupt, InterruptedError)) else "failed",
                      error=str(exc) or type(exc).__name__, finished_at_utc=utc())
        write_json(output / "report.json", report)
        raise


def remote_run(args, output):
    if args.reuse_attempt:
        raise ValueError("Run --reuse-attempt on the Pi holding the saved captures")
    host = args.host
    token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "_" + os.urandom(4).hex()
    remote = f"/home/protopi5/armfarm_calibration/{token}"
    python = "/home/protopi5/miniforge3/envs/lerobot/bin/python"
    options = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=3"]
    ssh = ["ssh", *options, host]
    subprocess.run([*ssh, "mkdir -p " + shlex.quote(remote + "/output")], check=True)
    subprocess.run(["scp", *options, "-q", str(Path(__file__)), f"{host}:{remote}/calibrate_workspace.py"], check=True)
    subprocess.run(["scp", *options, "-q", str(args.home), f"{host}:{remote}/home_pose.json"], check=True)
    command = [python, "-u", remote + "/calibrate_workspace.py", "--worker", "--output", remote + "/output", "--home", remote + "/home_pose.json"]
    if args.check:
        command.append("--check")
    code = 1
    process = subprocess.Popen([*ssh, shlex.join(command)])
    try:
        code = process.wait()
    except KeyboardInterrupt:
        stop = "import os,signal,pathlib; p=pathlib.Path(" + repr(remote + "/output/process.pid") + "); os.kill(int(p.read_text()),signal.SIGTERM) if p.exists() else None"
        subprocess.run([*ssh, shlex.join([python, "-c", stop])], check=False)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
        code = 130
    finally:
        copied = subprocess.run(["scp", *options, "-q", "-r", f"{host}:{remote}/output/.", str(output)], check=False)
        if copied.returncode:
            print(f"Copy failed; results retained at {host}:{remote}/output", flush=True)
            code = code or 1
        else:
            print(f"Saved on the laptop: {output}", flush=True)
    return code


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Read-only hardware check")
    parser.add_argument("--output", type=Path, help="Directory for this run's results")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--home", type=Path, default=HERE.parent / "home_pose.json")
    parser.add_argument("--reuse-attempt", type=Path, help="Reuse this Pi's saved captures; move only to home")
    args = parser.parse_args(argv)
    args.host = "protopi5"
    args.port = PORT
    args.frames_directory = "/dev/shm/so101_camd"
    args.trajectory = None
    args.duration = 4.
    args.gripper_open = 60.
    args.pattern = [9, 7]
    args.square_mm = 20.
    args.capture_timeout = 5.
    args.cameras = ["top"]
    return args


def main(argv=None):
    args = parse_args(argv)
    if os.environ.get("ARMFARM_SETTINGS"):
        settings = read_json(Path(os.environ["ARMFARM_SETTINGS"]))
        top = settings.get("cameras", {}).get("top")
        if isinstance(top, str) and top != "auto":
            CAMERAS["top"] = top
    output = (args.output or HERE.parent / "calibration_runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError("Output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    if os.name == "nt" and not args.worker:
        return remote_run(args, output)
    if os.name != "posix":
        raise ValueError("Local hardware execution requires Linux")
    cv2.setNumThreads(1)
    def stop(signum, frame):
        raise InterruptedError(f"Stopped by signal {signum}; last servo target remains holding")
    for name in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop)
    with tempfile.TemporaryDirectory(prefix="armfarm_assets_") as directory:
        args.trajectory = Path(directory) / "placement.json"
        args.trajectory.write_bytes(zlib.decompress(base64.b85decode(PLACEMENT_B85)))
        with process_lock(), camera_source(args):
            pid = output / "process.pid"
            pid.write_text(str(os.getpid()), encoding="ascii")
            try:
                return run_local(args, output)
            finally:
                pid.unlink(missing_ok=True)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("Stopped; torque remains at the last commanded position.")
    except Exception as exc:
        sys.exit(f"Calibration failed: {exc}")
