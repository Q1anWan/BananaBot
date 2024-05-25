syms phi0(t) L0(t) phi1(t) phi2(t) phi3(t) phi4(t) phi_dot_1 phi_dot_4 l1 l2 l3 l4 l5
x_B = l1*cos(phi1);
y_B = l1*sin(phi1);
x_C = x_B+l2*cos(phi2);
y_C = y_B+l2*sin(phi2);
x_D = l5+l4*cos(phi4);
y_D = l4*sin(phi4);
x_dot_B = diff(x_B,t);
y_dot_B = diff(y_B,t);
x_dot_C = diff(x_C,t);
y_dot_C = diff(y_C,t);
x_dot_D = diff(x_D,t);
y_dot_D = diff(y_D,t);

phi_dot_2 = ((x_dot_D-x_dot_B)*cos(phi3)+(y_dot_D-y_dot_B)*sin(phi3))/l2/sin(phi3-phi2);

x_dot_C = subs(x_dot_C,diff(phi2,t),phi_dot_2);
x_dot_C = subs(x_dot_C,...
    [diff(phi1,t),diff(phi4,t)],...
    [phi_dot_1,phi_dot_4]);
y_dot_C = subs(y_dot_C,diff(phi2,t),phi_dot_2);
y_dot_C = subs(y_dot_C,...
    [diff(phi1,t),diff(phi4,t)],...
    [phi_dot_1,phi_dot_4]);

x_dot = [x_dot_C; y_dot_C];
q_dot = [phi_dot_1; phi_dot_4];
x_dot = simplify(collect(x_dot,q_dot));
J = simplify(jacobian(x_dot,q_dot));

R = [cos(phi0-pi/2) -sin(phi0-pi/2);
     sin(phi0-pi/2)  cos(phi0-pi/2)];
M = [0 -1/L0;
     1     0];

JTRM = simplify(J.' * R * M);

MTRTJ = simplify(M.' * R.'* J);

fprintf('J^TRM MAT:\n');
pretty(JTRM);
fprintf('\ninv(J^TRM) MAT:\n')
pretty(simplify(inv(JTRM)));
fprintf('\nM^TR^TJ MAT:\n');
pretty(MTRTJ);  
