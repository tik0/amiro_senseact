%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de> 
% Functionity: Creates an rsb listener which listens on the '/' scope and
%              displays a LASE array in a plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
% close all;
clc; clear java

%% Add the java classes
PWD = pwd;
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11.0.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rstsandbox-0.11.9.jar'])
addpath('./ClusterAlgorithms')
addpath('./functions')

% Converter registration
rsb.matlab.ConverterRegistration.register('rst.claas.LASE_2000D_226Type', 'LASE_2000D_226')

% Create the listener
factory = rsb.Factory.getInstance();
listener = factory.createListener('/sense/LASE2000D/1');

% Create the queue with an 100 element buffer
scanData = rsb.matlab.LASE_2000D_226Queue(100,true);
listener.activate()
listener.addHandler(scanData, true)


while(1)
    figure(1)
    % Blocks 1 seconds if no data arives
    data = scanData.take(int32(1000));
    % Check if data has arrived
    if (~isempty(data))
        createFeatureVector( ...
            cell2mat(cell(data.getDistanceList.toArray)), ...
            cell2mat(cell(data.getPulsewidthList.toArray)));
    end
end

listener.deactivate();