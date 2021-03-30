function [corrSamDensity,samDensity,cumCentrality] = F_currentDensity(sensor,mesh,mode)
%[achievedSampling,achievedDensity,achievedCentrality] = F_currentDensity(sensor,mesh,mode)
%computes the corrected sampling density, the cumulative sampling density 
%and the cumulative centrality factor for the points reconstructed
%geometry.
%
%   Inputs:
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
%       mesh - structured array containing the reconstructed geometry. This
%              has the following elemets:
%           mesh.vertices - [nx3 double] mesh vertices coordinates
%           mesh.faces - [mx3 double] indices of triangles vertices
%           mesh.facesCentres - [mx3 double] barycentres of triangles
%           mesh.facesNormals - [mx3 double] unitary face normals
%           mesh.verticesNormals - [nx3 double] unitary point normals
%           mesh.facesArea - [mx1 double] face areas
%           mesh.verticesArea - [nx1 double] equivalent point areas
%           mesh.totalArea - [1 double] total reconstructed area
%       mode - [string] processing mode ('vertices' or 'faces') 
%
%   Outputs:
%       corrSamDensity - [mx1 if mode=faces or nx1 if mode=vertices]
%                        corrected cumulative sampling density
%       samDensity - [mx1 if mode=faces or nx1 if mode=vertices]
%                    cumulative sampling density
%       cumCentrality - [mx1 if mode=faces or nx1 if mode=vertices]
%                       cumulative centrality factor
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

vertices = mesh.vertices;
faces = mesh.faces;

if strcmp(mode,'faces')
    points = mesh.facesCentres;
    normals = mesh.facesNormals;
elseif strcmp(mode,'vertices')
    points = mesh.vertices;
    normals = mesh.verticesNormals;
end

%quiver3(points(:,1),points(:,2),points(:,3),normals(:,1),normals(:,2),normals(:,3))

samDens = zeros(size(points,1),1);
samCentrality = zeros(size(points,1),1);

nLocations = length(sensor);

for i = 1:nLocations
    sensorPosition = sensor{i}.position;
    sensorRotM = sensor{i}.rotationMatrix;
    
    lines = [points(:,1)-sensorPosition(1) points(:,2)-sensorPosition(2) points(:,3)-sensorPosition(3)];
    rotLines = lines*sensorRotM;
    
    %save('rayCastingDevelopment.mat','sensorPosition','lines','vertices','faces');
    
%     plot3(rotLines(:,1),rotLines(:,2),rotLines(:,3),'.');axis image
    
%     tic
    [~, distFromOrigin, ~] = F_intersectMultipleLineMesh(sensorPosition,lines,vertices,faces);
%     toc
%      
%     tic
%     [~,distFromOrigin,~,~,~] = mesh.tree.intersect(repmat(sensorPosition',1,size(lines,1)),lines');
%     distFromOrigin = distFromOrigin./sqrt(sum(lines.^2,2));
%     toc
    
    if strcmp(sensor{i}.type,'cartesian')
        a = repmat(-sensorRotM(:,3)',size(normals,1),1);
    end

    %quiver3(points(:,1),points(:,2),points(:,3),a(:,1),a(:,2),a(:,3))
    
    b = normals;
    ab = (a(:,1).*b(:,1))+(a(:,2).*b(:,2))+(a(:,3).*b(:,3));
    ma = sqrt(sum(a.^2,2));
    mb = sqrt(sum(b.^2,2));
    cosAngle = ab./(ma.*mb);
    
    centrFact = F_centralityFactor(sensor{i},rotLines);
    %centrFact(:) = 1;
    
    intensityFact = F_intensityFactor(sensor{i},rotLines);
    
    insideFrame = centrFact > 0;
    intersectingMesh = distFromOrigin >= (1-(1000*eps('single')));
    positiveCos = cosAngle > 0;
    
    iSampledPoints = intersectingMesh & insideFrame & positiveCos;
    
    samplingDensity = intensityFact(iSampledPoints);
    samplingDensity = samplingDensity.*cosAngle(iSampledPoints);
    
    %scatter3(rotLines(iSampledPoints,1),rotLines(iSampledPoints,2),rotLines(iSampledPoints,3),samplingDensity,samplingDensity,'filled')
    
    samDens(iSampledPoints) = samDens(iSampledPoints) + samplingDensity;
    samCentrality(iSampledPoints) = max([samCentrality(iSampledPoints) centrFact(iSampledPoints)],[],2);
    corrSamDensity = samDens.*samCentrality;
    cumCentrality = samCentrality;
    samDensity = samDens;
end

%------------- END CODE --------------
end