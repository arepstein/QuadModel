clc; clf; close all; clear all;
%quad dynamics, one body, one leg
%design variables
quad.leg=5.64/1000; %mm - leg length, straight line
quad.height=5.5/1000; %mm - body height
world.g=9.81; %mm s^-2 - gravity acceleration
quad.mass=1.5/1000; %g - body weight
magnet.torque=10/1000/1000; %mN-mm - magnetic torque applied to leg
world.time=[0:0.001:1]'; %s - time
magnet.frequency=5; %Hz - frequency of actuation
force.body=zeros(length(world.time),3);


%limits
theta.ground.one=asin((quad.height/2)/quad.leg);
theta.ground.two=pi-theta.ground.one*2;

%Theta creation
theta.leg=zeros(length(world.time),1);
theta.time=magnet.frequency*2*pi*world.time;
for i=1:length(world.time)
    if floor(theta.time(i)/(2*pi)) >= 1
        theta.leg(i,1)=((theta.time(i)-floor(theta.time(i)/(2*pi))*2*pi));
    else
        theta.leg(i,1)=theta.time(i);
    end
end

for j=1:length(world.time)
    force.body(j,:)=([-sin(theta.leg(j,1)) cos(theta.leg(j,1));...
                    0 0;...
                    -cos(theta.leg(j,1)) sin(theta.leg(j,1))]*...
                    [world.g*quad.mass; magnet.torque/quad.leg])';
end

figure(1)
plot(world.time,force.body(:,1),world.time,force.body(:,2),world.time,force.body(:,3))


options=odeset('RelTol',1e-6,'AbsTol',1e-10)
int=zeros(2,1);
quad_vars={quad world magnet theta};
[t,xy]=ode113(@(t,xy) quad_EOM(t,xy,quad_vars),...
    world.time,int,options);

figure(2)
plot(t,xy(:,1))%,t,xy(:,3))