clear all; close all; clc; clf;
speed_vector=zeros(13,1);
freq_vector=zeros(13,1);
for l=1:12;
    frequency=2+(1*l);
    freq_vector(l,1)=frequency;
    speed_vector(l,1)=full_velocity_function(frequency);
end
%%
figure(4)
plot(freq_vector,speed_vector,'-o')


%  exp_freq=[4.88 6.25 7.4 9.1 10 11.12 14.28 16.66 19.04];
%  exp_speed=[23.04732869	29.2013829	31.84449405	28.00417325	29.54278075	43.35521363	46.26139212	37.19996439	38.24431818];
%  exp_error=[0.319353154	1.386748397	2.127690389	3.238071574	1.958068783	1.972728653	3.5066222	2.258567497	5.906476148];
%  figure(4)
%  errorbar(exp_freq(1:end-1),exp_speed(1:end-1),exp_error(1:end-1))
%  
%  hold off