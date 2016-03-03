function R=RotMat(x)
if ~isvector(x)
    error('Input must be a vector')
end
if length(x)~=3
    error('Input must be a 3-dimensional vector')
end
Rx_a=[1 0 0; 0 cos(x(1)) sin(x(1)); 0 -sin(x(1)) cos(x(1))];
Ry_b=[cos(x(2)) 0 sin(x(2)); 0 1 0; -sin(x(2)) 0 cos(x(2))];
Rz_c=[cos(x(3)) sin(x(3)) 0; -sin(x(3)) cos(x(3)) 0; 0 0 1];

R=Rx_a*Ry_b*Rz_c;
end