function [ ] = plotAllAngles( queue )
%PLOT1D Summary of this function goes here
%   Detailed explanation goes here
a = zeros(8,200)
while(1)
a(:,1:end - 1) = a(:,2:end);
% blocks until new data is available
b = angles.take(10000)
c = parseArray(b);
a(:,end) = c;
angles.getQueue.clear;
plot(a');
getframe;
end
end

