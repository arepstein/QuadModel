       p1 =  -4.854e-10;
       p2 =  -3.434e-08;
       p3 =   2.893e-06;
       p4 =   0.0002324;
       x=X-80; %X is the global x
torque = p1*x^3 + p2*x^2 + p3*x + p4; %how much torque is applied to quad
