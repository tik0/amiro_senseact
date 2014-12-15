%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de> 
% Functionity: Creates an rsb listener and send a string "example
%              payload" to the scope "scope"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc

%% Add the java classes
PWD = pwd;
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rst-fleximon-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rst-sandbox-fleximon-0.11-SNAPSHOT.jar'])

%% Do the RSB stuff
% factory = rsb.Factory.getInstance();
% listener = factory.createListener('/scope');
% listener.activate();
% % listener.addHandler();
% listener.deactivate();

anglesPlotter('/')