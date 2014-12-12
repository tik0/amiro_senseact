function [] = anglesPlotter( scope )
%ANGLESPLOTTER Simple live plot of joint angles
%   Scope must map to a rst.kinematics.JointAngles type
angles = rsb.matlab.JointAnglesQueue;
createListener(scope, angles);
plotAngles(angles);
end

