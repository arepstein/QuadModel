function S=skewsymmat(x)
if ~isvector(x)
    error('Input must be a vector')
end
if length(x)~=3
    error('Input must be a 3-dimensional vector')
end
S=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end