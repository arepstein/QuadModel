xx=0:60;
xxx=xx';
magnet.torque=-0.1365*xxx.^2 + 5.4592*xxx + 165.45; %mN-mm
matrixtorque=[magnet.torque magnet.torque magnet.torque];
y=[-1;0;1];

figure(6)
plot(xxx,magnet.torque)
xlabel('Distance (mm)')
ylabel('Torque (mN-mm)')

figure(7)
contourf(xxx,y,matrixtorque',20)


figure(9)
hold on
img = imread('colorbar2.png');     %# Load a sample image
xImage = [0 60; 0 60];   %# The x data for the image corners
yImage = [-1 -1; 1 1];             %# The y data for the image corners
zImage = [0 0; 0 0];   %# The z data for the image corners
surf(xImage,yImage,zImage,...    %# Plot the surface
        'CData',img,...
     'FaceColor','texturemap');
scatter3(state(:,1),state(:,2),state(:,3))
axis([0 60 -1 1 0 2])
axis equal
%scatter3(state(:,1),state(:,2),state(:,3))
view(-20,15)
% 
% figure(8)
% hold on
% contourf(xxx,y,matrixtorque',20)
% scatter3(state(:,1),state(:,2),state(:,3))
% axis([0 60 -1 1 0 2])
% %axis equal
% %scatter3(state(:,1),state(:,2),state(:,3))
% view(-20,15)