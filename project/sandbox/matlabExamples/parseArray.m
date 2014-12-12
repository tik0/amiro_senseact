function [ matrix ] = parseArray( array )
%PARSEARRAY Parse Java array into Matlab matrix
%   Not nice. We need to find a better solution.
matrix = zeros(8,1);
for i = 1:8
    matrix(i) = array(i);
end
end

