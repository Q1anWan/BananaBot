function [L,Lp,Im,Ip] = VarCal(Leg_lenth,M,mp_board,mp_stator,Ibox,Istator,leg_width2,length_big,length_small)

Lp_board = Leg_lenth * length_big / (length_big+length_small); %板材质心到驱动轮质心距离

L = Lp_board*mp_board/(mp_board+mp_stator); %摆杆重心到驱动轮距离
Lp = Leg_lenth - L; %摆杆重心到机体转轴距离-机体转轴视作机体重心-质心到质心距

Lb2m = L*mp_stator/(mp_board+mp_stator); %摆杆重心到板材质心距离

%摆杆对自身重心转动惯量 平行轴定理
Ileg = (mp_board*(leg_width2*leg_width2 + Leg_lenth*Leg_lenth)/12) + Istator + mp_board*Lb2m*Lb2m + mp_stator*L*L;

%摆杆质心对(摆杆+机体)质心距离
Lp2m = Lp * M / (M + mp_board + mp_stator);
%机身质心对(摆杆+机体)质心距离
Lm2m = Lp * (mp_board + mp_stator) / (M + mp_board + mp_stator);

Im = Ibox  + M*Lp2m*Lp2m; %机体绕质心转动惯量
Ip = Ileg  + (mp_board+mp_stator)*Lm2m*Lm2m; %摆杆绕质心转动惯量



