function deviation = F_predictDensity(sensorPose,sensor,mesh,corrSamDensity,cumCentrality,iLowSampling,targetDensity,robotData,safetyDist,mode)
%deviation = F_predictDensity(sensorPose,sensor,mesh,corrSamDensity,cumCentrality,iLowSampling,targetDensity,robotData,safetyDist,mode)
%This is the objective function for the autonomous 3D reconstruction
%pipeline, described in the paper titled: "Autonomous 3D geometry
%reconstruction through robot-manipulated optical sensors", by 
%C. Mineo, D. Cerniglia, V. Ricotta and B. Reitinger. 
%
%   Inputs:
%       sensorPose - [1x6] pose of sensor
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
%       corrSamDensity - corrected sampling density
%       cumCentrality - cumulative centrality factor
%       iLowSampling - indices of points whose corrected sampling density
%                      is lower than the target density
%       targetDensity - target sampling density
%       robotData - robotData contains information about the sensor
%                   manipulator. If it is empty (robotData=[]), no
%                   kinematic constraint is considered. If pose
%                   reachbility must be checked, robotData must contain
%                   the following information:
%           robotData.Links: [6×1 double] (length of robot joints)
%           robotData.Joints: [6×3 double] (centre of kiematic joints)
%           robotData.robotPrefConf: (robot preferred configuration.
%                                    If 0 best configuration is
%                                    automatically determined)
%           robotData.toolParameters: [0 0 0 0 0 0] (tool parameters)
%       safetyDist - maximum distance of sensor from reconstructed geometry
%       mode - [string] processing mode ('vertices' or 'faces') 
%
%   Outputs:
%       deviation - deviation between theoretical number of points, required
%                   to sample the geometry, and acquired number of points 
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

vertices = mesh.vertices;
faces = mesh.faces;

if strcmp(mode,'faces')
    points = mesh.facesCentres(iLowSampling,:);
    normals = mesh.facesNormals(iLowSampling,:);
    areas = mesh.facesArea(iLowSampling);
elseif strcmp(mode,'vertices')
    points = mesh.vertices(iLowSampling,:);
    normals = mesh.verticesNormals(iLowSampling,:);
    areas = mesh.verticesArea(iLowSampling);
end

expectedPoints = sum(areas)*targetDensity;

corrSamDensity = corrSamDensity(iLowSampling);
cumCentrality = cumCentrality(iLowSampling);

newSampling = corrSamDensity;
newCentrality = zeros(size(newSampling));
rotM = F_eul2rotm(sensorPose(4:6),'rad');
normal = rotM(:,3)';
usedConf = 0;

%     currentAxes = gca;
%     axis(axesResult)
%     aXmax = fovX/2;
%     aYmax = fovY/2;
%     bestDepth = sqrt((resolution(1)*resolution(2))./(4.*tan(aXmax).*tan(aYmax).*desRes));
%     p = origin(1:3);
%     n = rotM(:,3);
%     u = rotM(:,1);
%     v = rotM(:,2);
%     t = [(p(1)+(n(1)*bestDepth)) (p(2)+(n(2)*bestDepth)) (p(3)+(n(3)*bestDepth))];
%     plot3([p(1) t(1)],[p(2) t(2)],[p(3) t(3)],'k.-','markersize',10);
%     quiver3(p(1),p(2),p(3),u(1),u(2),u(3),30,'r');
%     quiver3(p(1),p(2),p(3),v(1),v(2),v(3),30,'g');
%     quiver3(p(1),p(2),p(3),n(1),n(2),n(3),30,'b');
%     drawnow;
%     axis(currentAxes)

[~, distFromOrigin, ~] = F_intersectMultipleLineMesh(sensorPose(1:3),normal,vertices,faces);
%[~,distFromOrigin,~,~,~] = mesh.tree.intersect(origin(1:3)',normal');
distFromOrigin = distFromOrigin/sqrt(sum(normal.^2,2));
    
if (distFromOrigin > safetyDist)
    % Check if the pose is reachable by robot
    % robotPose = F_getRobotPose(origin(1:3),normal,0,robotData);
    if ~isempty(robotData)
        robotPose = sensorPose;
        robotPose(1,1:3) = robotPose(1,1:3) + (rotM(:,2)').*0;
        robotPose(1,1:3) = robotPose(1,1:3) + (rotM(:,1)').*(-17.5);
        robotPose(1,1:3) = robotPose(1,1:3) + (rotM(:,3)').*(-33.2);
        robotPose(4:6) = rad2deg(robotPose(4:6));
        [~,usedConf] = F_getConfSpaceTargets(robotPose,robotData);
    else
        usedConf = 1;
    end
end

lines = [points(:,1)-sensorPose(1) points(:,2)-sensorPose(2) points(:,3)-sensorPose(3)];
rotLines = lines*rotM;

% figure(10)
% quiver3(zeros(size(rotLines,1),1),zeros(size(rotLines,1),1),zeros(size(rotLines,1),1),rotLines(:,1),rotLines(:,2),rotLines(:,3),0);
% axis image;

centrFact = F_centralityFactor(sensor,rotLines);
%centrFact(:) = 1;

iValid = find(centrFact > 0);

lines = lines(iValid,:);
rotLines = rotLines(iValid,:);
normals = normals(iValid,:);

intensityFact = F_intensityFactor(sensor,rotLines);
[~, distFromOrigin, ~] = F_intersectMultipleLineMesh(sensorPose(1:3),lines,vertices,faces);
%[~,distFromOrigin,~,~,~] = mesh.tree.intersect(repmat(origin(1:3)',1,size(lines,1)),lines');
%distFromOrigin = distFromOrigin./sqrt(sum(lines.^2,2));


if strcmp(sensor.type,'cartesian')
    a = repmat(-normal,size(normals,1),1);
end

b = normals;
ab = (a(:,1).*b(:,1))+(a(:,2).*b(:,2))+(a(:,3).*b(:,3));
ma = sqrt(sum(a.^2,2));
mb = sqrt(sum(b.^2,2));
cosAngle = ab./(ma.*mb);

intersectingMesh = distFromOrigin >= (1-(1000*eps('single')));
positiveCos = cosAngle > 0;

if usedConf>0 && usedConf<7
    goodPoints = intersectingMesh & positiveCos;
    
    newSampling(iValid(goodPoints)) = corrSamDensity(iValid(goodPoints)) + (intensityFact(goodPoints).*cosAngle(goodPoints));
    newCentrality(iValid(goodPoints)) = max([cumCentrality(iValid(goodPoints)) centrFact(goodPoints)],[],2);
    newSampling(iValid(goodPoints)) = (newSampling(iValid(goodPoints)).*newCentrality(iValid(goodPoints)));
    
    newSampling(newSampling>targetDensity) = targetDensity;
    estimatedSampledPoints = sum(areas.*newSampling);
else
    estimatedSampledPoints = 0;
end

deviation = expectedPoints - estimatedSampledPoints;

%------------- END CODE --------------

end

