function [mPData,iPData] = F_generatePData(mRData,iRData,sensor,outputDir)
%[mPData,iPData] = F_generatePData(mRData,iRData,sensor,outputDir)
%generates processed data (point normals and sampling density)
%
%   Inputs:
%       mRData - address of memory-mapped raw data file
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
%       sensor - structured array containing sensor properties
%       outputDir - string containing path of output directory
%
%   Outputs:
%       mPData - address of memory-mapped processed data file
%       iPData - indices of processed data
%           iPData.n.x - indices of x component of unitary normal vector
%           iPData.n.y - indices of y component of unitary normal vector
%           iPData.n.z - indices of z component of unitary normal vector
%           iPData.n.xyz - indices of xyz components of unitary normal vector
%           iPData.sg - indices of sampling density (sampling goodness) 
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

resAz = sensor.resAz;
resEl = sensor.resEl;
    
procWriteID = fopen([outputDir '/procData.dat'], 'w');
%% Compute normals
fwrite(procWriteID,pcnormals(pointCloud([mRData.Data(iRData.c.x) mRData.Data(iRData.c.y) mRData.Data(iRData.c.z)])),'double');
mPData = memmapfile([outputDir '/procData.dat'],'Format','double','Writable',true);
nPoints = length(mPData.Data)/3;
iPData = [];
iPData.n = [];
iPData.n.x = uint64(1:nPoints)';
iPData.n.y = iPData.n.x + nPoints;
iPData.n.z = iPData.n.y + nPoints;
iPData.n.xyz = [iPData.n.x'; iPData.n.y'; iPData.n.z'];

%% Flip normals pointing to the wrong direction
d = sqrt(sum([mRData.Data(iRData.v.x) mRData.Data(iRData.v.y) mRData.Data(iRData.v.z)].^2,2));
cosTheta = ((mPData.Data(iPData.n.x).*(-mRData.Data(iRData.v.x))) +...
    (mPData.Data(iPData.n.y).*(-mRData.Data(iRData.v.y))) +...
    (mPData.Data(iPData.n.z).*(-mRData.Data(iRData.v.z))))./d;

isFlipped = cosTheta<0;
mPData.Data(iPData.n.xyz(:,isFlipped),:) = -mPData.Data(iPData.n.xyz(:,isFlipped),:);

%% Correct cosTheta
cosTheta(isFlipped) = -cosTheta(isFlipped);

%% Compute intensity factor
%surface = 4.*(d.^2).*tan(aXmax).*tan(aYmax);
if strcmp(sensor.type,'cartesian')
    aZmax = sensor.azRange(2);
    eLmax = sensor.elRange(2);
    if sensor.optimumDist < sensor.optimumDist_sampling
        fwrite(procWriteID,((resAz*resEl)./(4.*(d.^2).*tan(aZmax).*tan(eLmax))).*cosTheta.*(sensor.optimumDist/sensor.optimumDist_sampling),'double');
    else
        fwrite(procWriteID,((resAz*resEl)./(4.*(d.^2).*tan(aZmax).*tan(eLmax))).*cosTheta,'double');
    end
else
    aZmin = sensor.azRange(1);
    eLmin = sensor.elRange(1);
    aZmax = sensor.azRange(2);
    eLmax = sensor.elRange(2);
    if sensor.optimumDist < sensor.optimumDist_sampling
        fwrite(procWriteID,((resAz*resEl)./((d.^2).*(aZmax-aZmin).*(sin(eLmax)-sin(eLmin)))).*cosTheta.*(sensor.optimumDist/sensor.optimumDist_sampling),'double');
    else
        fwrite(procWriteID,((resAz*resEl)./((d.^2).*(aZmax-aZmin).*(sin(eLmax)-sin(eLmin)))).*cosTheta,'double');
    end
end
mPData = memmapfile([outputDir '/procData.dat'],'Format','double','Writable',false);
iPData.sg = iPData.n.z + nPoints;
fclose(procWriteID);

%------------- END CODE --------------
end

