function F_saveDsDataAsPLY(mRData,mPData,iRData,iPData,iDsData,outputDir)
%F_saveDsDataAsPLY(mRData,mPData,iRData,iPData,iDsData,outputDir)
%saves the down-sampled point cloud as PLY file, containing point normals
%and colors.
%
%   Inputs:
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
%       iDsData - indices of points belonging to the initial downsamples cloud
%       outputDir - string containing the path of the output directory
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------
pcwrite(pointCloud([mRData.Data(iRData.c.x(iDsData)) mRData.Data(iRData.c.y(iDsData)) mRData.Data(iRData.c.z(iDsData))],...
          'Normal',[mPData.Data(iPData.n.x(iDsData)) mPData.Data(iPData.n.y(iDsData)) mPData.Data(iPData.n.z(iDsData))],...
           'Color',[mRData.Data(iRData.col.R(iDsData)) mRData.Data(iRData.col.G(iDsData)) mRData.Data(iRData.col.B(iDsData))]),...
                   [outputDir '/dsData'],'PLYFormat','binary');
%------------- END CODE --------------
               
end

