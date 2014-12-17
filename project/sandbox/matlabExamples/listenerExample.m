%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de> 
% Functionity: Creates an rsb listener which listens on the '/' scope and
%              displays a 200x8 vector array in a plot
% Demo: Run " $ bag-play WalkFourLegsCCA2.tide"
%       and then this script with < r2012a
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; clear java

%% Add the java classes
PWD = pwd;
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rst-fleximon-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rst-sandbox-fleximon-0.11-SNAPSHOT.jar'])

% Converter registration
rsb.matlab.ConverterRegistration.register('rst.kinematics.JointAnglesType', 'JointAngles')

% Create the listener
factory = rsb.Factory.getInstance();
listener = factory.createListener('/');

% Create the queue with an 100 element buffer
angles = rsb.matlab.JointAnglesQueue(100,true);
listener.activate()
listener.addHandler(angles, true)

jointAngles = zeros(8,200);
while(1)
    jointAngles(1:end - 1) = jointAngles(2:end);
    % Blocks 10 seconds if no data arives
    data = angles.take(int32(10000));
    jointAngles(:,end) = [data(1), data(2), data(3), data(4), data(5), data(6), data(7), data(8)]';
    angles.getQueue.clear;
    plot(jointAngles');
    getframe;
end

listener.deactivate();