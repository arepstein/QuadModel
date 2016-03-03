function dstate=reduced_state_quad_EOM(t,state,quad_vars)
dstate=zeros(6,1);
%state=[u v r x y psi]' body velocities XYZ and body angular velocities

quad=quad_vars{1};
world=quad_vars{2};
magnet=quad_vars{3};
theta=quad_vars{4};
force=quad_vars{5};

%Generate theta profile
if floor(magnet.frequency*2*pi*t/(2*pi)) >= 1
    theta.general=((magnet.frequency*2*pi*t-...
        floor(magnet.frequency*2*pi*t/(2*pi))*2*pi));
else
    theta.general=magnet.frequency*2*pi*t;
end

%Describe rotation of each leg
theta.l1=theta.general;
theta.l2=theta.general;
theta.l3=theta.general;
theta.l4=theta.general;
%need to handle the offsets later

%body forces from leg
force.leg.l1=[-sin(theta.l1) -cos(theta.l1);...
              0 0;...
              -cos(theta.l1) sin(theta.l1)]*...
              [force.ground; force.spring];
force.leg.l2=[-sin(theta.l2) -cos(theta.l2);...
              0 0;...
              -cos(theta.l2) sin(theta.l2)]*...
              [force.ground; force.spring];
force.leg.l3=[-sin(theta.l3) -cos(theta.l3);...
              0 0;...
              -cos(theta.l3) sin(theta.l3)]*...
              [force.ground; force.spring];
force.leg.l4=[-sin(theta.l4) -cos(theta.l4);...
              0 0;...
              -cos(theta.l4) sin(theta.l4)]*...
              [force.ground; force.spring];
          
%body torques from leg
torque.leg.l1=skewsymmat(quad.leg.a1)*force.leg.l1;
torque.leg.l2=skewsymmat(quad.leg.a2)*force.leg.l2;
torque.leg.l3=skewsymmat(quad.leg.a3)*force.leg.l3;
torque.leg.l4=skewsymmat(quad.leg.a4)*force.leg.l4;

if theta.l1>=theta.limit && theta.l1<=pi-2*theta.limit
    bool.l1=1;
else
    bool.l1=0;
end
if theta.l2>=theta.limit && theta.l2<=pi-2*theta.limit
    bool.l2=1;
else
    bool.l2=0;
end
if theta.l3>=theta.limit && theta.l3<=pi-2*theta.limit
    bool.l3=1;
else
    bool.l3=0;
end
if theta.l4>=theta.limit && theta.l4<=pi-2*theta.limit
    bool.l4=1;
else
    bool.l4=0;
end

%Adding up body forces and torques
force.body.x=bool.l1*force.leg.l1(1,1)+bool.l2*force.leg.l2(1,1)+...
             bool.l3*force.leg.l3(1,1)+bool.l4*force.leg.l4(1,1);
force.body.y=bool.l1*force.leg.l1(2,1)+bool.l2*force.leg.l2(2,1)+...
             bool.l3*force.leg.l3(2,1)+bool.l4*force.leg.l4(2,1);
force.body.z=bool.l1*force.leg.l1(3,1)+bool.l2*force.leg.l2(3,1)+...
             bool.l3*force.leg.l3(3,1)+bool.l4*force.leg.l4(3,1);
torque.body.x=bool.l1*torque.leg.l1(1,1)+bool.l2*torque.leg.l2(1,1)+...
             bool.l3*torque.leg.l3(1,1)+bool.l4*torque.leg.l4(1,1);
torque.body.y=bool.l1*torque.leg.l1(2,1)+bool.l2*torque.leg.l2(2,1)+...
             bool.l3*torque.leg.l3(2,1)+bool.l4*torque.leg.l4(2,1);
torque.body.z=bool.l1*torque.leg.l1(3,1)+bool.l2*torque.leg.l2(3,1)+...
             bool.l3*torque.leg.l3(3,1)+bool.l4*torque.leg.l4(3,1);

%Equations of motion
dstate(1)=(force.body.x+quad.mass.magnitude*state(2)*state(3))/...
            quad.mass.magnitude;
dstate(2)=-1*state(1)*state(3);
dstate(3)=torque.body.z/quad.inertia.matrix(3,3);
dstate(4)=state(1)*cos(state(6))-state(2)*sin(state(6));
dstate(5)=state(1)*sin(state(6))+state(2)*cos(state(6));
dstate(6)=state(3);