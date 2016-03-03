function xydot=quad_EOM(t,xy,quad_vars)
xydot=zeros(2,1);

quad=quad_vars{1};
world=quad_vars{2};
magnet=quad_vars{3};
theta=quad_vars{4};

%Theta creation
theta.leg=magnet.frequency*2*pi*t;
% if floor(theta.time/(2*pi)) >= 1
%     theta.leg=((theta.time-floor(theta.time/(2*pi))*2*pi));
% else
%     theta.leg=theta.time;
% end

%Force creation
n=ceil(theta.leg/(2*pi));
if theta.leg>0 && theta.leg<=n*(theta.ground.two+theta.ground.one)
    force.y=(magnet.torque/(quad.leg))*sin(theta.leg);
    force.x=(magnet.torque/(quad.leg));%*cos(theta.leg);
else
    force.y=0;
    force.x=0;
end


xydot(1)=xy(2);
xydot(2)=force.x/quad.mass;
% xydot(3)=xy(4);
% xydot(4)=-1*world.g+force.y/quad.mass;

