function [areaFaces,areaVertices,totalArea] = F_meshArea(faces,vertices)
%[areaFaces,areaVertices,totalArea] = F_meshArea(meshFaces,meshVertices)
%This function gives the areas of the faces in a triangular mesh
%
%   Inputs:
%       faces - [nx3] indices of points mesh faces
%       vertices - [mx3] vertices of mesh
%
%   Outputs:
%       areaFaces - [nx1] area of faces
%       areaVertices - [mx1] equivalent area of vertices
%       totalArea - total mesh surface
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

P1 = vertices(faces(:,1),:);
P2 = vertices(faces(:,2),:);
P3 = vertices(faces(:,3),:);

% Length of edges
L = [sqrt(sum(((P1-P2).^2),2)) sqrt(sum(((P2-P3).^2),2)) sqrt(sum(((P3-P1).^2),2))];

% Area calculation with Heron's formula
s = ((L(:,1)+L(:,2)+L(:,3))./2);
areaFaces = sqrt(s.*(s-L(:,1)).*(s-L(:,2)).*(s-L(:,3)));

nV = size(vertices,1);
areaVertices = zeros(nV,1);
for i=1:nV
    iTri = (faces(:,1)==i) | (faces(:,2)==i) | (faces(:,3)==i);
    areaVertices(i) = sum(areaFaces(iTri))/3;
end

totalArea = sum(areaFaces);

%------------- END CODE --------------
end