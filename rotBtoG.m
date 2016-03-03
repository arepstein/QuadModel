function R=rotBtoG(x)
if ~isvector(x)
    error('Input must be a vector')
end
if length(x)~=3
    error('Input must be a 3-dimensional vector')
end
R=[cos(x(3))*cos(x(2)) -sin(x(3))*cos(x(1))+cos(x(3))*sin(x(2))*sin(x(1)) sin(x(3))*sin(x(1))+cos(x(3))*cos(x(1))*sin(x(2));...
   sin(x(3))*cos(x(2)) -cos(x(3))*cos(x(1))+sin(x(3))*sin(x(2))*sin(x(1)) -cos(x(3))*sin(x(1))+sin(x(3))*cos(x(1))*sin(x(2));...
   -sin(x(2)) cos(x(2))*sin(x(1)) cos(x(2))*cos(x(1))];
end