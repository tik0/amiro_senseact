function [ factory ] = addRSB(PWD)
%ADDRSX Add RSX libraries to Matlab
%   Detailed explanation goes here
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rst-fleximon-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rst-sandbox-fleximon-0.11-SNAPSHOT.jar'])
rsb.matlab.ConverterRegistration.register()
end