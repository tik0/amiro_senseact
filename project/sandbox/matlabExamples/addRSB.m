function [ factory ] = addRSB()
%ADDRSX Add RSX libraries to Matlab
%   Detailed explanation goes here
rootpath = '/vol/cit/share/java';
javaaddpath ([rootpath '/RSBJava-0.5.0.jar'])
javaaddpath /opt/local/share/java/protobuf.jar
javaaddpath ([rootpath '/rstsandbox.jar'])
javaaddpath ([rootpath '/rst.jar'])
javaaddpath /Users/swrede/Workspace/RSB.m/dist/lib/rsb.m-0.5.0.jar
rsb.matlab.ConverterRegistration.register()
end