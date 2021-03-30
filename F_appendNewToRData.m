function [mRData,iRData,iNewRData] = F_appendNewToRData(rawWriteID,newData,outputDir)
%[mRData,iRData,iNewRData] = F_appendNewToRData(rawWriteID,newData,outputDir)
%appends new frame data to raw data file.
%
%   Inputs:
%       rawWriteID - handle of file for storing the raw data
%       newData - new data
%       outputDir - string containing the path of the output directory
%
%   Outputs:
%       mRData - memory-mapping address of raw data file
%       iRData - indices of raw data
%           iRData.c.x - indices of all x coordinates
%           iRData.c.y - indices of all y coordinates
%           iRData.c.z - indices of all z coordinates
%           iRData.v.x - indices of all x coordinates of unitary view vector
%           iRData.v.y - indices of all y coordinates of unitary view vector
%           iRData.v.z - indices of all z coordinates of unitary view vector
%           iRData.col.R - indices of all R componets of point color
%           iRData.col.G - indices of all G componets of point color
%           iRData.col.B - indices of all B componets of point color
%       iNewRData - indices of new points
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------
% Append data to file
fwrite(rawWriteID,newData','double');

% Map file to memory
mRData = memmapfile([outputDir '/rawData.dat'],'Format','double','Writable',false);

% Output indices
nPoints = length(mRData.Data);
iRData = [];
iRData.c = [];
iRData.c.x = uint64(1:9:nPoints)';
iRData.c.y = iRData.c.x + 1;
iRData.c.z = iRData.c.y + 1;

iRData.v = [];
iRData.v.x = iRData.c.x + 3;
iRData.v.y = iRData.c.y + 3;
iRData.v.z = iRData.c.z + 3;

iRData.col = [];
iRData.col.R = iRData.c.x + 6;
iRData.col.G = iRData.c.y + 6;
iRData.col.B = iRData.c.z + 6;

if (length(iRData.c.x)-size(newData,1))>0
    oldCount = uint64(length(iRData.c.x)-size(newData,1));
else
    oldCount = uint64(0);
end
if size(newData,1)>0
    iNewRData = uint64((oldCount+1):length(iRData.c.x))';
else
    iNewRData = uint64(0);
end
%------------- END CODE --------------

end

