function [] = rmvRSB(PWD)
%RMVRSX Remove RSX libraries from Java path
%   Detailed explanation goes here
javarmpath([PWD '/rst-sandbox-fleximon-0.11-SNAPSHOT.jar'])
javarmpath([PWD '/rst-fleximon-0.11-SNAPSHOT.jar'])
javarmpath([PWD '/protobuf-java-2.5.0.jar'])
javarmpath([PWD '/rsb-matlab-0.11-SNAPSHOT.jar'])
javarmpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
% clear java
end
