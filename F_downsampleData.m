function [iDsData,iClusters] = F_downsampleData(mRData,mPData,iRData,iNewRData,iPData,iDsData,dsCubeSide,dsMode)
%[iDsData,iClusters] = F_downsampleData(mRData,mPData,iRData,iNewRData,iPData,iDsData,dsCubeSide,dsMode)
%operates downsampling and merging of new point cloud with initial point
%cloud.
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
%       iNewRData - indices of new points
%       iPData - indices of processed data
%           iPData.n.x - indices of x component of unitary normal vector
%           iPData.n.y - indices of y component of unitary normal vector
%           iPData.n.z - indices of z component of unitary normal vector
%           iPData.n.xyz - indices of xyz components of unitary normal vector
%           iPData.sg - indices of sampling density (sampling goodness) 
%       iDsData - indices of points belonging to the initial downsamples cloud
%       dsCubeSide - size of side of down-sampling cube
%       dsMode - [string] down-sampling mode ('best', 'worst' or 'average') 
%
%   Outputs:
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
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

% Find grid occupancy indices
if ~isempty(iDsData)
    oldCount = iDsData;
    %cubeIndices = [mRData.Data(iRData.c.x(oldCount))-origin(1) mRData.Data(iRData.c.y(oldCount))-origin(2) mRData.Data(iRData.c.z(oldCount))-origin(3)];
    cubeIndices = [mRData.Data(iRData.c.x(oldCount)) mRData.Data(iRData.c.y(oldCount)) mRData.Data(iRData.c.z(oldCount))];
else
    oldCount = 0;
    cubeIndices = zeros(0,3);
end
%cubeIndices = cubeIndices + (gridSpacing/2);
cubeIndices = cubeIndices./dsCubeSide;
cubeIndicesOld = int32(cubeIndices);
[sortedCubeIndicesOld,IsOld] = sortrows(cubeIndicesOld,'ascend');

if iNewRData(end)>0
    %cubeIndices = [mRData.Data(iRData.c.x(iNewRData))-origin(1) mRData.Data(iRData.c.y(iNewRData))-origin(2) mRData.Data(iRData.c.z(iNewRData))-origin(3)];
    cubeIndices = [mRData.Data(iRData.c.x(iNewRData)) mRData.Data(iRData.c.y(iNewRData)) mRData.Data(iRData.c.z(iNewRData))];
else
    cubeIndices = zeros(0,3);
end
%cubeIndices = cubeIndices + (gridSpacing/2);
cubeIndices = cubeIndices./dsCubeSide;
cubeIndicesNew = int32(cubeIndices);
[sortedCubeIndicesNew,IsNew] = sortrows(cubeIndicesNew,'ascend');

[sortedSharedCubesIndices,~,~] = intersect(sortedCubeIndicesOld,sortedCubeIndicesNew,'rows');

% Find indices of cubes occupied by points of the old cloud but not
% by points of the new cloud
[iCubesOldOnly,~] = ismember(sortedCubeIndicesOld,sortedSharedCubesIndices,'rows');
IsOldOnly = IsOld(~iCubesOldOnly);
iCubesOldOnly = sortedCubeIndicesOld(~iCubesOldOnly,:);
if isempty(IsOldOnly)
    iPointsOldOnly = [];
else
    iPointsOldOnly = 1:length(IsOldOnly);
    iPointsOldOnly = iPointsOldOnly';
end

% Find indices of cubes occupied by points of the new cloud but not
% by points of the old cloud
[iCubesNewOnly,~] = ismember(sortedCubeIndicesNew,sortedSharedCubesIndices,'rows');
sortedCubeIndicesNewOnly = sortedCubeIndicesNew(~iCubesNewOnly,:);
IsNewOnly = IsNew(~iCubesNewOnly,:);
[iCubesNewOnly,IfirstNewOnly,~] = unique(sortedCubeIndicesNewOnly,'rows','first');
[~,IlastNewOnly,~] = unique(sortedCubeIndicesNewOnly,'rows','last');
iPointsNewOnly = [IfirstNewOnly IlastNewOnly];

% Find indices of shared cubes
[iCubesShared,IfirstOld] = ismember(sortedSharedCubesIndices,sortedCubeIndicesOld,'rows');
iCubesShared = sortedSharedCubesIndices(iCubesShared,:);
[~,IlastOld] = ismember(sortedSharedCubesIndices,flipud(sortedCubeIndicesOld),'rows');
flippedIndices = fliplr(1:length(oldCount))';
IlastOld = flippedIndices(IlastOld);
iPointsOldShared = [IfirstOld IlastOld];

[~,IfirstNew] = ismember(sortedSharedCubesIndices,sortedCubeIndicesNew,'rows');
[~,IlastNew] = ismember(sortedSharedCubesIndices,flipud(sortedCubeIndicesNew),'rows');
flippedIndices = fliplr(1:length(iNewRData))';
IlastNew = flippedIndices(IlastNew);
iPointsNewShared = [IfirstNew IlastNew];

iClusters = [];
iClusters.iCubes = [];
iClusters.iCubes.iOldOnly = iCubesOldOnly;
iClusters.iCubes.iNewOnly = iCubesNewOnly;
iClusters.iCubes.iShared = iCubesShared;
iClusters.iPoints = [];
iClusters.iPoints.addressOldOnly = oldCount(IsOldOnly);
iClusters.iPoints.addressNewOnly = iNewRData(IsNewOnly);
iClusters.iPoints.addressOldShared = oldCount(IsOld);
iClusters.iPoints.addressNewShared = iNewRData(IsNew);
iClusters.iPoints.iOldOnly = iPointsOldOnly;
iClusters.iPoints.iNewOnly = iPointsNewOnly;
iClusters.iPoints.iOldShared = iPointsOldShared;
iClusters.iPoints.iNewShared = iPointsNewShared;

nMerged = length(IfirstOld);
nOldOnly = length(iDsData) - nMerged;
nNewOnly = length(IfirstNewOnly);

nMergedPoints = nOldOnly + nMerged + nNewOnly;

iDsData = uint64(zeros(nMergedPoints,1));
if nOldOnly>0
    iDsData(1:nOldOnly) = oldCount(IsOldOnly);
end

if nMerged>0
    switch dsMode
        case 'random'
            parfor i = (nOldOnly + 1):(nOldOnly + nMerged)
                counter = i-nOldOnly;
                indPointsOld = IsOld(IfirstOld(counter):IlastOld(counter));
                indPointsNew = IsNew(IfirstNew(counter):IlastNew(counter));
                n = length(indPointsOld) + length(indPointsNew);
                n = ceil(rand*n);
                if n<=length(indPointsOld)
                    iDsData(i) = oldCount(indPointsOld(n));
                else
                    n = n-length(indPointsOld);
                    iDsData(i) = iNewRData(indPointsNew(n));
                end
            end
        case 'best'
            parfor i = (nOldOnly + 1):(nOldOnly + nMerged)
                counter = i-nOldOnly;
                indPointsOld = IsOld(IfirstOld(counter):IlastOld(counter));
                indPointsNew = IsNew(IfirstNew(counter):IlastNew(counter));
                quality = [mPData.Data(iPData.sg(oldCount(indPointsOld))); mPData.Data(iPData.sg(iNewRData(indPointsNew)))];
                ind = find(quality == max(quality),1);
                if ind<=length(indPointsOld)
                    iDsData(i) = oldCount(indPointsOld(ind));
                else
                    ind = ind - length(indPointsOld);
                    iDsData(i) = iNewRData(indPointsNew(ind));
                end
            end
        case 'worst'
            parfor i = (nOldOnly + 1):(nOldOnly + nMerged)
                counter = i-nOldOnly;
                indPointsOld = IsOld(IfirstOld(counter):IlastOld(counter));
                indPointsNew = IsNew(IfirstNew(counter):IlastNew(counter));
                quality = [mPData.Data(iPData.sg(oldCount(indPointsOld))); mPData.Data(iPData.sg(iNewRData(indPointsNew)))];
                ind = find(quality == min(quality),1);
                if ind<=length(indPointsOld)
                    iDsData(i) = oldCount(indPointsOld(ind));
                else
                    ind = ind - length(indPointsOld);
                    iDsData(i) = iNewRData(indPointsNew(ind));
                end
            end
    end
end

if nNewOnly>0
    switch dsMode
        case 'random'
            parfor i = (nOldOnly + nMerged + 1):nMergedPoints
                counter = i-(nOldOnly + nMerged);
                indPointsNew = IsNewOnly(IfirstNewOnly(counter):IlastNewOnly(counter));
                n = length(indPointsNew);
                n = ceil(rand*n);
                iDsData(i) = iNewRData(indPointsNew(n));
            end
        case 'best'
            parfor i = (nOldOnly + nMerged + 1):nMergedPoints
                counter = i-(nOldOnly + nMerged);
                indPointsNew = IsNewOnly(IfirstNewOnly(counter):IlastNewOnly(counter));
                quality = mPData.Data(iPData.sg(iNewRData(indPointsNew)));
                ind = find(quality == max(quality),1);
                iDsData(i) = iNewRData(indPointsNew(ind));
            end
        case 'worst'
            parfor i = (nOldOnly + nMerged + 1):nMergedPoints
                counter = i-(nOldOnly + nMerged);
                indPointsNew = IsNewOnly(IfirstNewOnly(counter):IlastNewOnly(counter));
                quality = mPData.Data(iPData.sg(iNewRData(indPointsNew)));
                ind = find(quality == min(quality),1);
                iDsData(i) = iNewRData(indPointsNew(ind));
            end
    end
end

%------------- END CODE --------------

end

