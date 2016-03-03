clc; clf; close all; clear all;

Quad_Variables

%Magnetic parameters, torque can be adjusted by adjusting height,...
    %as per Abbott paper, frequency is dependent upon actuator speed
magnet.torque=80; %mN-mm - magnetic torque applied to leg
magnet.frequency=4; %Hz - frequency of actuation
magnet.force=3.5; %mN - magnetic force, attraction between magnets

int=zeros(12,1);
% options=odeset('RelTol',1e-5,'AbsTol',1e-7);
quad_vars={quad world magnet theta force};
[t,state]=ode45(@(t,state) full_state_quad_EOM(t,state,quad_vars),...
    [0 1],int);
figure(1)
plot(t,state(:,1),t,state(:,2),t,state(:,3))
figure(2)
plot(t,state(:,4),t,state(:,5),t,state(:,6))
figure(3)
plot(t,state(:,7),t,state(:,8),t,state(:,9))
figure(4)
plot(t,state(:,10),t,state(:,11),t,state(:,12))
figure(5)
scatter3(state(:,1),state(:,2),state(:,3))
axis equal
view(-20,15)