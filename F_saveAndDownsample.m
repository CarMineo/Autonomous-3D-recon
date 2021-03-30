function [rawWriteID,fDetWriteID,mRData,mPData,iRData,iPData,iDsData,iClusters] = F_saveAndDownsample(rawWriteID,fDetWriteID,newData,iDsData,sensor,dsCubeSide,dsMode,outputDir)
%[rawWriteID,fDetWriteID,mRData,mPData,iRData,iPData,iDsData,iClusters] = F_saveAndDownsample(rawWriteID,fDetWriteID,newData,iDsData,sensor,dsCubeSide,dsMode,outputDir)
%This function operates saving of incoming data, merging and down-sampling
%using the algorithm described in the paper titled: "Autonomous 3D 
%geometry reconstruction through robot-manipulated optical sensors", by 
%C. Mineo, D. Cerniglia, V. Ricotta and B. Reitinger.
%It makes use of memory-mapping to avoid filling the computer random access
%memory and make the framework scalable.
%
%   Inputs:
%       rawWriteID - handle of file for storing the raw data
%       fDetWriteID - handle of file for storing the details
%       newData - new data
%       iDsData - indices of points belonging to the initial downsamples cloud
%       sensor - structured array containing the sensor details
%           sensor.position - Initial pose Cartesian coordinates
%           sensor.rotationMatrix - Initial pose rotation matrix
%           sensor.resAz - Sensor azimutal resolution
%           sensor.resEl - Sensor elavation resolution
%           sensor.azRange - Azimutal angle range
%           sensor.elRange - Elevation angle range
%           sensor.range - Sensor depth range [min max]
%           sensor.optimumDist_accuracy - Sensor accuracy-dependant stand-off
%           sensor.optimumDist_sampling - Sensor sampling-dependant stand-off
%           sensor.optimumDist - Sensor optimum stand-off
%           sensor.type = 'cartesian' - Sensor type ('cartesian' or 'polar')
%       dsCubeSide - size of side of down-sampling cube
%       dsMode - [string] down-sampling mode ('best', 'worst' or 'average') 
%       outputDir - string containing the path of the output directory
%   
%   Outputs:
%       rawWriteID - handle of file for storing the raw data
%       fDetWriteID - handle of file for storing the details
%       mRData - address of memory-mapped raw data file
%       mPData - address of memory-mapped processed data file
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
%       iPData - indices of processed data
%           iPData.n.x - indices of x component of unitary normal vector
%           iPData.n.y - indices of y component of unitary normal vector
%           iPData.n.z - indices of z component of unitary normal vector
%           iPData.n.xyz - indices of xyz components of unitary normal vector
%           iPData.sg - indices of sampling density (sampling goodness) 
%       iDsData - indices of points belonging to the updated down-sampled
%                 and merged point cloud
%       iClusters - indices of points belonging to the down-sampling cubes
%           iClusters.iCubes.iOldOnly
%           iClusters.iCubes.iNewOnly
%           iClusters.iCubes.iShared
%           iClusters.iPoints.addressOldOnly
%           iClusters.iPoints.addressNewOnly
%           iClusters.iPoints.addressOldShared
%           iClusters.iPoints.addressNewShared
%           iClusters.iPoints.iOldOnly
%           iClusters.iPoints.iNewOnly
%           iClusters.iPoints.iOldShared
%           iClusters.iPoints.iNewShared
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

% Append new points (with view vector and colors) to mapped rawData file
[mRData,iRData,iNewRData] = F_appendNewToRData(rawWriteID,newData,outputDir);

% Append details of new data frame to file (fDetWriteID)
F_appendNewFrameDetToFile(fDetWriteID,iNewRData,sensor);

% Process data to generate normals and sampling goodness
mPData = [];
[mPData,iPData] = F_generatePData(mRData,iRData,sensor,outputDir);

% Find indices of downsampled data
[iDsData,iClusters] = F_downsampleData(mRData,mPData,iRData,iNewRData,iPData,iDsData,dsCubeSide,dsMode);

% Save indices of downsampled data
F_saveIndDsData(outputDir,iDsData)

% Save downsampled data as PLY file
F_saveDsDataAsPLY(mRData,mPData,iRData,iPData,iDsData,outputDir)

%------------- END CODE --------------
end

