clear;
fprintf('\n\t//-K in Arm Math Matrix Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15\n')

% Warning: Calculated as a whole body, 2 legs are simulated in 1.
g =	    9.81;  %重力加速度(SZ)
M =  	2.561;  %机体质量 //4*4310 + 16*铝方 + 板材 + 电池 + 其他电气 = 4*300 + 16*6 + 730 + 385 + 50 = 2461 g
mw = 	0.754;  %驱动轮转子质量 // 2*(4310转子 + 轮) 
mp_board  = 	0.322;  %摆杆总质量=mp_boadr+mp_stator //2*(小腿*2 + 大腿*2 + 轴承 + 电机定子) = 2 * (49g*2 + 29*2 + 5g + 150g) = 652 g
mp_stator = 	0.300;

Iw =	0.00172753692;  %驱动轮转子转动惯量 // 2*(电机转子 + 轮毂板材 + 包胶轮) = 2*(0.00002176346 + 0.000138088 + 0.000703917) = 0.00172753692 kg/m^2
R =	    0.063;   %驱动轮半径
l = 	0.02;    %机体重心到其转轴距离
length_big = 0.120;%大腿长度
length_small = 0.218;%小腿长度
leg_width = 0.06;%腿板材宽度*2，用于计算转动惯量

Ibox = 0.012133164; % 机身转动惯量 模块分解后平行轴定理加和计算
Istator = 0.00093117308; %两侧定子转动惯量 4310圆柱-电机转子惯量

%Used for sim
x0=[0,0,0,0,0,0];
ConstA = [0,0,0,1,0,0];

BasicData=[g,M/2,R,l,mw,mp_board,Iw];

Q = [80 0 0 0 0 0; 0 20 0 0 0 0; 0 0 20 0 0 0; 0 0 0 10 0 0; 0 0 0 0 90 0; 0 0 0 0 0 40]; %权重矩阵 Q 的设计
%R = [100 0; 0 80]; %权重矩阵 R 的设计
%R = [150 0; 0 6.2]; %权重矩阵 R 的设计
R = [70 0; 0 500];

%腿长200mm    
Leg_lenth = 0.200;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.210;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.220;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.230;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.240;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.250;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.260;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.270;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.280;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.290;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))

Leg_lenth = 0.300;

[L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width,length_big,length_small);
VarData = [L,Lp,Im,Ip];
[K,A,B,C,D] = LQRFun2(BasicData,VarData,Q,R);

fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',Leg_lenth,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',Leg_lenth,R(1,1),R(2,2),-K(2,1),-K(2,2))