syms x y z rho pi gamma u v w p q r
XYZ=transpose([x y z]);
EUL=transpose([rho pi gamma]);
body=transpose([u v w p q r]);
J=[rotBtoG(XYZ) zeros(3,3); zeros(3,3) angular_rotBtoG(EUL)];
J*body