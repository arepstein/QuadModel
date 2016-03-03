%design variables
quad.height=5.5; %mm - body height
quad.width=12.5; %mm - body width
quad.length=20; %mm - body length
quad.leg.length=5.64; %mm - leg length, straight line
quad.leg.a1=0.5*[quad.length quad.width 0]';
quad.leg.a2=0.5*[quad.length -quad.width 0]';
quad.leg.a3=0.5*[-quad.length quad.width 0]';
quad.leg.a4=-0.5*[quad.length quad.width 0]';

quad.mass.magnitude=1.5*2; %g - body weight
quad.mass.matrix=quad.mass.magnitude*eye(3); %g - body weight
quad.inertia.matrix=[quad.mass.magnitude/12*(quad.height^2+quad.width^2) 0 0;...
                    0 quad.mass.magnitude/12*(quad.height^2+quad.length^2) 0;...
                    0 0 quad.mass.magnitude/12*(quad.width^2+quad.length^2)];
                    %g-mm^2

%World-space parameters
world.g=9.81*0; %m s^-2 - gravity acceleration
world.time=[0:0.00001:4]'; %s - time

%Magnetic parameters, torque can be adjusted by adjusting height,...
    %as per Abbott paper, frequency is dependent upon actuator speed
magnet.torque=80; %mN-mm - magnetic torque applied to leg
magnet.frequency=10; %Hz - frequency of actuation
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
theta.offset.l3=0;
theta.offset.l4=0;

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


%Ground forces
force.ground=magnet.torque/quad.leg.length; %mN
force.spring=.01*force.ground; %mN - make small contribution to upward force
force.friction=0.8*quad.mass.magnitude*world.g/4; %mN - static friction...
                                                  %force when body is on
                                                  %contact with ground, per
                                                  %leg. 

%limit when leg is in contact with ground
%theta.limit=asin((.5*quad.height)/quad.leg.length);
theta.limit=pi;
%body forces from leg
for j=1:length(world.time)
force.leg.l1(:,j)=[-sin(theta.l1(j,1)) -cos(theta.l1(j,1));...
              0 0;...
              -cos(theta.l1(j,1)) sin(theta.l1(j,1))]*...
              [force.ground; force.spring];
force.leg.l2(:,j)=[-sin(theta.l2(j,1)) -cos(theta.l2(j,1));...
              0 0;...
              -cos(theta.l2(j,1)) sin(theta.l2(j,1))]*...
              [force.ground; force.spring];
force.leg.l3(:,j)=[-sin(theta.l3(j,1)) -cos(theta.l3(j,1));...
              0 0;...
              -cos(theta.l3(j,1)) sin(theta.l3(j,1))]*...
              [force.ground; force.spring];
force.leg.l4(:,j)=[-sin(theta.l4(j,1)) -cos(theta.l4(j,1));...
              0 0;...
              -cos(theta.l4(j,1)) sin(theta.l4(j,1))]*...
              [force.ground; force.spring];
          
%body torques from leg
torque.leg.l1(:,j)=skewsymmat(quad.leg.a1)*force.leg.l1(:,j);
torque.leg.l2(:,j)=skewsymmat(quad.leg.a2)*force.leg.l2(:,j);
torque.leg.l3(:,j)=skewsymmat(quad.leg.a3)*force.leg.l3(:,j);
torque.leg.l4(:,j)=skewsymmat(quad.leg.a4)*force.leg.l4(:,j);
end


