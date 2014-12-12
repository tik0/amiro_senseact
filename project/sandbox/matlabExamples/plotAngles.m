function [ ] = plotAngles( queue )
%PLOT1D Summary of this function goes here
%   Detailed explanation goes here
a = zeros(8,200)
while(1)
a(1:end - 1) = a(2:end);
% blocks until new data is available
a(end) = queue.take;
queue.getQueue.clear;
plot(a);
getframe;
end
end

