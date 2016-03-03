function dstate=full_state_quad_EOM(t,state,quad_vars)
dstate=zeros(12,1);
%state=[u v w p q r]' body velocities XYZ and body angular velocities
%state=[x y z rho pi gamma] next 6 states

quad=quad_vars{1};
world=quad_vars{2};
magnet=quad_vars{3};
theta=quad_vars{4};
force=quad_vars{5};

%magnet.torque=-0.1365*state(1)^2 + 5.4592*state(1) + 165.45; %mN-mm
% magnet.torque=80; %mN-mm
% magnet.force=3.5*0; %mN - magnetic force, attraction between magnets
%Ground forces
force.ground=magnet.torque/quad.leg.length; %mN
force.spring=.01*force.ground*0; %mN - make small contribution to upward force
force.friction=0.8*quad.mass.magnitude*world.g/4*0; %mN - static friction...
                                                  %force when body is on
                                                  %contact with ground, per
                                                  %leg. 

%Generate theta profile
if floor(magnet.frequency*2*pi*t/(2*pi)) >= 1
    theta.general=((magnet.frequency*2*pi*t-...
        floor(magnet.frequency*2*pi*t/(2*pi))*2*pi));
else
    theta.general=magnet.frequency*2*pi*t;
end

%Describe rotation of each leg
theta.l1=theta.general;

if theta.offset.l2 ~= 0
    for i=1:length(world.time)
        if floor((theta.general+theta.offset.l2)/(2*pi)) >= 1
            theta.l2=((theta.general+theta.offset.l2-...
                floor((theta.general+theta.offset.l2)/(2*pi))*2*pi));
        else
            theta.l2=theta.general+theta.offset.l2;
        end
    end
else
    theta.l2=theta.general;
end

if theta.offset.l3 ~= 0
    for i=1:length(world.time)
        if floor((theta.general+theta.offset.l3)/(2*pi)) >= 1
            theta.l3=((theta.general+theta.offset.l3-...
                floor((theta.general+theta.offset.l3)/(2*pi))*2*pi));
        else
            theta.l3=theta.general+theta.offset.l3;
        end
    end
else
    theta.l3=theta.general;
end

if theta.offset.l4 ~= 0
    if floor((theta.general+theta.offset.l4)/(2*pi)) >= 1
            theta.l4=((theta.general+theta.offset.l4-...
                floor((theta.general+theta.offset.l4)/(2*pi))*2*pi));
    else
            theta.l4=theta.general+theta.offset.l4;
    end
else
    theta.l4=theta.general;
end

if theta.l1>=1*theta.limit && theta.l1<=2*theta.limit
    bool.l1=1;
else
    bool.l1=0;
end
if theta.l2>=1*theta.limit && theta.l2<=2*theta.limit
    bool.l2=1;
else
    bool.l2=0;
end
if theta.l3>=1*theta.limit && theta.l3<=2*theta.limit
    bool.l3=1;
else
    bool.l3=0;
end
if theta.l4>=1*theta.limit && theta.l4<=2*theta.limit
    bool.l4=1;
else
    bool.l4=0;
end

%body forces from leg
force.leg.l1=[-sin(theta.l1) -cos(theta.l1);...
              0 0;...
              -cos(theta.l1) sin(theta.l1)]*...
              [bool.l1*force.ground+force.friction*(1-bool.l1); force.spring*bool.l1+magnet.force*0];
force.leg.l2=[-sin(theta.l2) -cos(theta.l2);...
              0 0;...
              -cos(theta.l2) sin(theta.l2)]*...
              [bool.l2*force.ground+force.friction*(1-bool.l2); force.spring*bool.l2+magnet.force*0];
force.leg.l3=[-sin(theta.l3) -cos(theta.l3);...
              0 0;...
              -cos(theta.l3) sin(theta.l3)]*...
              [bool.l3*force.ground+force.friction*(1-bool.l3); force.spring*bool.l3+magnet.force*0];
force.leg.l4=[-sin(theta.l4) -cos(theta.l4);...
              0 0;...
              -cos(theta.l4) sin(theta.l4)]*...
              [bool.l4*force.ground+force.friction*(1-bool.l4); force.spring*bool.l4+magnet.force*0];
          
%body torques from leg
torque.leg.l1=skewsymmat(quad.leg.a1)*force.leg.l1;
torque.leg.l2=skewsymmat(quad.leg.a2)*force.leg.l2;
torque.leg.l3=skewsymmat(quad.leg.a3)*force.leg.l3;
torque.leg.l4=skewsymmat(quad.leg.a4)*force.leg.l4;

%Adding up body forces and torques
force.body.x=force.leg.l1(1,1)+force.leg.l2(1,1)+...
             force.leg.l3(1,1)+force.leg.l4(1,1);
force.body.y=force.leg.l1(2,1)+force.leg.l2(2,1)+...
             force.leg.l3(2,1)+force.leg.l4(2,1);
force.body.z=force.leg.l1(3,1)+force.leg.l2(3,1)+...
             force.leg.l3(3,1)+force.leg.l4(3,1);
torque.body.x=torque.leg.l1(1,1)+torque.leg.l2(1,1)+...
             torque.leg.l3(1,1)+torque.leg.l4(1,1);
torque.body.y=torque.leg.l1(2,1)+torque.leg.l2(2,1)+...
             torque.leg.l3(2,1)+torque.leg.l4(2,1);
torque.body.z=torque.leg.l1(3,1)+torque.leg.l2(3,1)+...
             torque.leg.l3(3,1)+torque.leg.l4(3,1);
% if force.body.x==0 && state(8)>0
%     force.body.x=-0.8*quad.mass.magnitude*world.g;
% end

%Equations of motion
dstate(1)=(force.body.x-quad.mass.magnitude*state(3)*state(5)...
          +quad.mass.magnitude*state(2)*state(6)-...
          quad.mass.magnitude*world.g*1*sin(state(11)))/quad.mass.magnitude;
dstate(2)=(force.body.y+quad.mass.magnitude*state(3)*state(4)...
          -quad.mass.magnitude*state(1)*state(6)+...
          quad.mass.magnitude*world.g*1*sin(state(10))*cos(state(11)))/quad.mass.magnitude;  
dstate(3)=(force.body.z-quad.mass.magnitude*state(2)*state(4)...
          +quad.mass.magnitude*state(1)*state(5)+...
          quad.mass.magnitude*world.g*1*cos(state(10))*cos(state(11)))/quad.mass.magnitude;  
dstate(4)=(torque.body.x-quad.inertia.matrix(3,3)*state(5)*state(6)...
          +quad.inertia.matrix(2,2)*state(5)*state(6))/quad.inertia.matrix(1,1);  
dstate(5)=(torque.body.y+(quad.inertia.matrix(3,3)-quad.inertia.matrix(1,1))*...
          state(4)*state(6))/quad.inertia.matrix(2,2);
dstate(6)=(torque.body.z+(quad.inertia.matrix(1,1)-quad.inertia.matrix(2,2))*...
          state(4)*state(5))/quad.inertia.matrix(3,3);
dstate(7)=state(3)*(sin(state(7))*sin(state(9))+cos(state(7))*cos(state(9))*sin(state(8)))...
          -state(2)*(cos(state(7))*sin(state(9))-cos(state(9))*sin(state(7))*sin(state(8)))...
          +state(1)*cos(state(8))*cos(state(9));
dstate(8)=state(1)*cos(state(8))*sin(state(9))-state(3)*(cos(state(9))*sin(state(7))...
          - cos(state(7))*sin(state(8))*sin(state(9)))-state(2)*(cos(state(7))*cos(state(9))...
          - sin(state(7))*sin(state(8))*sin(state(9)));
dstate(9)=state(3)*cos(state(7))*cos(state(8))-state(1)*sin(state(8))...
          +state(2)*cos(state(8))*sin(state(7));
dstate(10)=state(4);
dstate(11)=state(5)*cos(state(10))-state(6)*sin(state(10));
dstate(12)=-state(6)*cos(state(10))-state(5)*sin(state(10));

