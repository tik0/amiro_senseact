%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de> 
% Functionity: Creates an rsb listener which listens on the '/' scope and
%              displays a LASE array in a plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; clear java

%% Add the java classes
PWD = pwd;
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11.0.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rstsandbox-0.11.9.jar'])

% Converter registration
rsb.matlab.ConverterRegistration.register('rst.claas.LASE_2000D_226Type', 'LASE_2000D_226')

% Create the listener
factory = rsb.Factory.getInstance();
listener = factory.createListener('/LASE');

% Create the queue with an 100 element buffer
scanData = rsb.matlab.LASE_2000D_226Queue(100,true);
listener.activate()
listener.addHandler(scanData, true)

% Constants
deltaAngle = 90; % millidegree
rangeAngle = 90000; % millidegree
startAngle = -45000; % millidegree
angles = startAngle + deltaAngle : deltaAngle : startAngle + rangeAngle;

while(1)
    % Blocks 1 seconds if no data arives
    data = scanData.take(int32(1000));
    datetime
    if (~isempty(data))
        % Distances
        subplot(2,1,1)
        distances = cell2mat(cell(data.getDistanceList.toArray));
        distances(distances == hex2dec('7FFFFFFF')) = nan; % To noisy
        distances(distances == -hex2dec('80000000')) = nan; % To near
        scatter(angles, distances);
        xlabel('Winkel [m°]')
        ylabel('Distanz [1/10 mm]')
        % Pulsewidth
        subplot(2,1,2)
        pulsewidth = cell2mat(cell(data.getPulsewidthList.toArray));
        scatter(angles, pulsewidth);
        xlabel('Winkel [m°]')
        ylabel('Pulsweite')
        getframe;
    end
end

listener.deactivate();
