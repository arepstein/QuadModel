clear all; close all; clc;
%World-space parameters
world.g=9.81; %m s^-2 - gravity acceleration
world.time=[0:0.00001:1]'; %s - time

%Magnetic parameters, torque can be adjusted by adjusting height,...
    %as per Abbott paper, frequency is dependent upon actuator speed
magnet.torque=80; %mN-mm - magnetic torque applied to leg
magnet.frequency=4; %Hz - frequency of actuation
magnet.force=3.5; %mN - magnetic force, attraction between magnets

%Leg rotation
%Theta creation
theta.general=zeros(length(world.time),1);
for i=1:length(world.time)
    if floor(magnet.frequency*2*pi*world.time(i)/(2*pi)) >= 1
        theta.general(i,1)=((magnet.frequency*2*pi*world.time(i)-floor(magnet.frequency*2*pi*world.time(i)/(2*pi))*2*pi));
    else
        theta.general(i,1)=magnet.frequency*2*pi*world.time(i);
    end
end

theta.offset.l1=0; %No offset, other legs relative to leg 1
theta.offset.l2=0;
theta.offset.l3=pi/2;
theta.offset.l4=pi/4;

%Describe rotation of each leg
theta.l1=theta.general;

if theta.offset.l2 ~= 0
    for i=1:length(world.time)
        if floor((theta.general(i,1)+theta.offset.l2)/(2*pi)) >= 1
            theta.l2(i,1)=((theta.general(i)+theta.offset.l2-...
                floor((theta.general(i,1)+theta.offset.l2)/(2*pi))*2*pi));
        else
            theta.l2(i,1)=theta.general(i,1)+theta.offset.l2;
        end
    end
else
    theta.l2=theta.general;
end

if theta.offset.l3 ~= 0
    for i=1:length(world.time)
        if floor((theta.general(i,1)+theta.offset.l3)/(2*pi)) >= 1
            theta.l3(i,1)=((theta.general(i)+theta.offset.l3-...
                floor((theta.general(i,1)+theta.offset.l3)/(2*pi))*2*pi));
        else
            theta.l3(i,1)=theta.general(i,1)+theta.offset.l3;
        end
    end
else
    theta.l3=theta.general;
end

if theta.offset.l4 ~= 0
    for i=1:length(world.time)
        if floor((theta.general(i,1)+theta.offset.l4)/(2*pi)) >= 1
            theta.l4(i,1)=((theta.general(i)+theta.offset.l4-...
                floor((theta.general(i,1)+theta.offset.l4)/(2*pi))*2*pi));
        else
            theta.l4(i,1)=theta.general(i,1)+theta.offset.l4;
        end
    end
else
    theta.l4=theta.general;
end

%Find reset points of the legs, e.g. discontinuities 
intervals.l1=0:1/(2*magnet.frequency):world.time(end);
tzeros.l1=zeros(length(intervals.l1),1);

if theta.offset.l2 ~= 0
    intervals.l2=intervals.l1(1,2:end)-(theta.offset.l2/(2*pi))/magnet.frequency;
    tzeros.l2=zeros(length(intervals.l2),1);
else
    intervals.l2=intervals.l1;
    tzeros.l2=tzeros.l1;
end

if theta.offset.l3 ~= 0
    intervals.l3=intervals.l1(1,2:end)-(theta.offset.l3/(2*pi))/magnet.frequency;
    tzeros.l3=zeros(length(intervals.l3),1);
else
    intervals.l3=intervals.l1;
    tzeros.l3=tzeros.l1;
end

if theta.offset.l4 ~= 0
    intervals.l4=intervals.l1(1,2:end)-(theta.offset.l4/(2*pi))/magnet.frequency;
    tzeros.l4=zeros(length(intervals.l4),1);
else
    intervals.l4=intervals.l1;
    tzeros.l4=tzeros.l1;
end

TimeIntervals=unique(sort([intervals.l1 intervals.l2 intervals.l3 intervals.l4]));
plot(TimeIntervals,zeros(length(TimeIntervals),1),'o',world.time,theta.l1,world.time,theta.l3,world.time,theta.l4)


