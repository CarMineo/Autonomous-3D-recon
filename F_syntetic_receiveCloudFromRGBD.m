function [newData,intersectedFaces] = F_syntetic_receiveCloudFromRGBD(sensor,vertices,faces)
%[newData,intersectedFaces] = F_syntetic_receiveCloudFromRGBD(sensor,vertices,faces)
%This function generates a syntetic sensor point cloud, through
%intersecating the sensor rays with the triangular mesh of a given object.
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
%       vertices - [mx3] vertices of mesh
%       faces - [px3] indices of points mesh faces
%
%   Outputs:
%       newData - array containing new point cloud
%           newData = [newCloud.Location newCloud.DetectionVector newCloud.color];
%       intersectedFaces - indices of intersected mesh faces
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

positionRGBD = sensor.position;
R = sensor.rotationMatrix;
resAz = sensor.resAz;
resEl = sensor.resEl;
% tree = opcodemesh(sampleMeshPoints',sampleMeshConnections');

angleX = sensor.azRange(2)-sensor.azRange(1);
angleY = sensor.elRange(2)-sensor.elRange(1);

pixelPosX = 0.5:1:resAz-0.5;
anglePosX = atan(tan(0.5*angleX)*((2*(fliplr(pixelPosX))/(resAz))-1));

pixelPosY = 0.5:1:resEl-0.5;
anglePosY = atan(tan(0.5*angleY)*((2*(fliplr(pixelPosY))/(resEl))-1));

[aX,aY] = meshgrid(anglePosX,anglePosY);
% aMax = max([abs(aX(:)) abs(aY(:))],[],2);
aMax = abs(aX(:).*aY(:));
aMax = aMax./max(aMax);

% aMax = reshape(aMax,size(aX));
% figure(10);
% subplot(1,3,1); imshow(aX);colorbar;
% subplot(1,3,2); imshow(aY);colorbar;
% subplot(1,3,3); imshow(aMax);colorbar;

ux = tan(anglePosX);
uy = tan(anglePosY);
[uX,uY] = meshgrid(ux,uy);
uZ = ones(size(uX));

lines = [uX(:) uY(:) uZ(:)]/R;
normLines = sqrt(sum(lines.^2,2));
lines = lines./normLines;

% tic
[intersectionPoints, distFromOrigin, intersectedFaces] = F_intersectMultipleLineMesh(positionRGBD,lines,vertices, faces);
% toc

% tic
% [~,distFromOrigin,intersectedFaces,~,intersectionPoints] = tree.intersect(repmat(positionRGBD',1,size(lines,1)),lines');
% intersectionPoints = intersectionPoints';
% toc

indices = find(~isnan(distFromOrigin));
newCloud.Location = intersectionPoints(indices,:);

lines = [newCloud.Location(:,1)-positionRGBD(1) newCloud.Location(:,2)-positionRGBD(2) newCloud.Location(:,3)-positionRGBD(3)];
rotLines = lines*R;
d = rotLines(:,3);
    
newCloud.DetectionVector = repmat(R(:,3)',size(d,1),1);
newCloud.DetectionVector = [newCloud.DetectionVector(:,1).*d newCloud.DetectionVector(:,2).*d newCloud.DetectionVector(:,3).*d];
   
newCloud.color = zeros(size(newCloud.DetectionVector));
newCloud.color(:,2) = 255;
newData = [newCloud.Location newCloud.DetectionVector newCloud.color];

% quiver3(newData(:,1),newData(:,2),newData(:,3),newData(:,4),newData(:,5),newData(:,6),0);

%------------- END CODE --------------

end

