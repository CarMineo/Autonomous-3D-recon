function F_appendNewFrameDetToFile(fDetWriteID,iNewRData,sensor)
%F_appendNewFrameDetToFile(fDetWriteID,iNewRData,sensor) appends new frame
%details to file.
%
%   Inputs:
%       fDetWriteID - handle of file for storing the details
%       iNewRData - indices of new raw data
%       sensor - structured array containing the sensor properties
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b

%------------- BEGIN CODE --------------

fwrite(fDetWriteID,iNewRData(1),'uint64');
fwrite(fDetWriteID,iNewRData(end),'uint64');
fwrite(fDetWriteID,sensor.resAz,'uint32');
fwrite(fDetWriteID,sensor.resEl,'uint32');
fwrite(fDetWriteID,sensor.azRange,'double');
fwrite(fDetWriteID,sensor.elRange,'double');
fwrite(fDetWriteID,sensor.position,'double');
fwrite(fDetWriteID,sensor.rotationMatrix,'double');
fwrite(fDetWriteID,sensor.optimumDist,'double');

%------------- END CODE --------------

end

