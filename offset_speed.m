clear all; close all; clc; clf;
speed_vector=zeros(32,1);
offset_vector=zeros(32,1);
for l=1:32;
    offset=l*pi/16;
    offset_vector(l,1)=offset;
    speed_vector(l,1)=full_offset_sweep_function(offset);
end
%%
figure(4)
% hold on
plot(offset_vector,speed_vector,'-o')
figure(5)
plot(offset_vector*180/pi,speed_vector,'-o')
axis([0 360 51 59])

% exp_freq=[4.88 6.25 7.4 9.1 10 11.12 14.28 16.66 19.04];
% exp_speed=[23.04732869	29.2013829	31.84449405	28.00417325	29.54278075	43.35521363	46.26139212	37.19996439	38.24431818];
% exp_error=[0.319353154	1.386748397	2.127690389	3.238071574	1.958068783	1.972728653	3.5066222	2.258567497	5.906476148];
% 
% errorbar(exp_freq(1:end-1),exp_speed(1:end-1),exp_error(1:end-1))
% 
% hold off