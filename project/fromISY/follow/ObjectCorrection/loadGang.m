%% Import data from text file.
% Script for importing data from the following text file:
%
%    /homes/mbarther/git/murox_dev/project/AMiRo/follow/ObjectCorrection/RingProximityAndOdometry_Right.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2015/01/05 15:40:13

%% Initialize variables.
filename = 'RingProximityAndOdometry_Gang.txt';
delimiter = '\t';
startRow = 2;
endRow = 142;

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow-startRow+1, 'Delimiter', delimiter, 'HeaderLines', startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
PRV0 = dataArray{:, 1};
PRV1 = dataArray{:, 2};
PRV2 = dataArray{:, 3};
PRV3 = dataArray{:, 4};
PRV4 = dataArray{:, 5};
PRV5 = dataArray{:, 6};
PRV6 = dataArray{:, 7};
PRV7 = dataArray{:, 8};
Xym = dataArray{:, 9};
Yym = dataArray{:, 10};
Thetayrad = dataArray{:, 11};

%% Clear temporary variables
clearvars filename delimiter startRow endRow formatSpec fileID dataArray ans;