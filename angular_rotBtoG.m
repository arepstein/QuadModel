function T=angular_rotBtoG(x)
if ~isvector(x)
    error('Input must be a vector')
end
if length(x)~=3
    error('Input must be a 3-dimensional vector')
end
T=[1 sin(x(1))*tan(x(2)) cos(x(1))*tan(x(2));...
   0 cos(x(1)) -sin(x(1));...
   0 sin(x(1))/cos(x(2)) cos(x(1))/cos(x(2))];
end