function [] = rmvRSB()
%RMVRSX Remove RSX libraries from Java path
%   Detailed explanation goes here
javarmpath /public/amarsi_workshop/lib/rsb.m-0.5.0.jar
javarmpath /usr/share/java/rst.jar
javarmpath /usr/share/java/rstsandbox.jar
javarmpath /usr/share/java/RSBJava-0.5.0.jar
javarmpath /usr/share/java/protobuf.jar
clear java
end
