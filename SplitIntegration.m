clc; clf; close all; clear all;

Quad_Variables

t_cell=cell(length(TimeIntervals)-1,1);
state_cell=cell(length(TimeIntervals)-1,1);

%Magnetic parameters, torque can be adjusted by adjusting height,...
    %as per Abbott paper, frequency is dependent upon actuator speed
magnet.torque=80; %mN-mm - magnetic torque applied to leg
%magnet.frequency=5; %Hz - frequency of actuation
magnet.force=3.5; %mN - magnetic force, attraction between magnets

int=zeros(12,1);
quad_vars={quad world magnet theta force};

for index=1:length(TimeIntervals)-1
    [t,state]=ode45(@(t,state) full_state_quad_EOMv2(t,state,quad_vars),...
        [TimeIntervals(index) TimeIntervals(index+1)],int);
    t_cell{index}=t;
    state_cell{index}=state;
    int=state(end,:);
end

time=cell2mat(t_cell);
states=cell2mat(state_cell);

 
figure(1)
plot(time,states(:,1),time,states(:,2),time,states(:,3))
figure(2)
plot(time,states(:,4),time,states(:,5),time,states(:,6))
figure(3)
plot(time,states(:,7),time,states(:,8),time,states(:,9))
figure(4)
plot(time,states(:,10),time,states(:,11),time,states(:,12))
figure(5)
scatter3(states(:,1),states(:,2),states(:,3))
axis equal
view(-20,15)
figure(6)
scatter3(states(:,7),states(:,8),states(:,9))
axis equal
view(-20,15)