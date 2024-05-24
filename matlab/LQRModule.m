%Author, NUAA: WANG JIAN.
clear;
syms theta x phi T Tp N P Nm Pm Nf
syms R L Lm l mw mp M Iw Ip Im

syms theta_dot theta_dot_2 phi_dot phi_dot_2 
syms x_dot_2 xb_dot_2
syms g 

x_dot_2=xb_dot_2-(L+Lm)*cos(theta)*theta_dot_2+(L+Lm)*sin(theta)*theta_dot^2;
N = Nm + mp*( x_dot_2 + L*(theta_dot_2*cos(theta) - theta_dot^2*sin(theta)) );
P = Pm + mp*g - mp*L*(theta_dot_2*sin(theta) + theta_dot^2*cos(theta)  );
Nm = M*( x_dot_2 + (L+Lm)*(theta_dot_2*cos(theta)-theta_dot^2*sin(theta)) - l*(phi_dot_2*cos(phi) - phi_dot^2*sin(phi)) );
Pm = M*g - M*( (L+Lm)*(theta_dot_2*sin(theta) + theta_dot^2*cos(theta)) + l*(phi_dot_2*sin(phi) + phi_dot^2*cos(phi)) );


eqA=x_dot_2==(T-N*R)/(Iw/R+mw*R);
eqB=Ip*theta_dot_2==((P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp);
eqC=Im*phi_dot_2==(Tp+Nm*l*cos(phi)+Pm*l*sin(phi));

[xb_dot_2,theta_dot_2,phi_dot_2]=solve([eqA,eqB,eqC],[xb_dot_2,theta_dot_2,phi_dot_2]);
%mod_sol=solve(eqA,eqB,eqC,[xb_dot_2,theta_dot_2,phi_dot_2]);

xb_dot_2=simplify(collect(xb_dot_2));
theta_dot_2=simplify(collect(theta_dot_2));
phi_dot_2=simplify(collect(phi_dot_2));

A1=diff(theta_dot_2,theta);
A2=diff(theta_dot_2,phi);
B1=diff(theta_dot_2,T);
B2=diff(theta_dot_2,Tp);

A3=diff(x_dot_2,theta);
A4=diff(x_dot_2,phi);
B3=diff(x_dot_2,T);
B4=diff(x_dot_2,Tp);

A5=diff(phi_dot_2,theta);
A6=diff(phi_dot_2,phi);
B5=diff(phi_dot_2,T);
B6=diff(phi_dot_2,Tp);

A1=subs(A1,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
A2=subs(A2,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
B1=subs(B1,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
B2=subs(B2,{theta,phi,theta_dot,phi_dot},{0,0,0,0});

A3=subs(A3,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
A4=subs(A4,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
B3=subs(B3,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
B4=subs(B4,{theta,phi,theta_dot,phi_dot},{0,0,0,0});

A5=subs(A5,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
A6=subs(A6,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
B5=subs(B5,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
B6=subs(B6,{theta,phi,theta_dot,phi_dot},{0,0,0,0});
