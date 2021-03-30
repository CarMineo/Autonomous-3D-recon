function [bTri,nTri] = F_getTriangleNormals(Nodes,faces)
%[bTri,nTri] = F_getTriangleNormals(Nodes,iSurfaces)
%   The function computes the normals to all triangles. "Nodes" is a nx3
%   array containing the n vertices of the mesh (with no redundancies).
%   "iSurfaces" is a cell array of 3xnT uint32 arrays containing the
%   indexed triangles of the mesh surfaces. Every cell contains one surface. 
%   "bTri" and "nTri" are respectively the baricentres of the triangles (to
%   be used as origins of the normals) and the triangles' normals.
%   The baricentres and the normals are divided into cells (depending on 
%   the surface they belong to).
%
%   Author:         University of Strathclyde
%                   Centre for Ultrasonic Engineering
%                   Dr Carmelo Mineo
%
%   Last uodate: 06/07/2016

x = Nodes(:,1);
y = Nodes(:,2);
z = Nodes(:,3);

cellArray = 0;

if iscell(faces)
    cellArray = 1;
    ns = numel(faces);
    nTri = cell(1,ns);
    bTri = cell(1,ns);
else
    ns = 1;
    nTri = [];
    bTri = [];
end

for i=1:ns
    if cellArray
        tri = faces{i};
    else
        tri = faces;
    end
    
    numTri = size(tri,1);
    
    % The cross product of two sides of the triangle equals the surface normal.
    % So, if V = P2 - P1 and W = P3 - P1
    
    V1 = [(x(tri(:,2)) - x(tri(:,1))) (y(tri(:,2)) - y(tri(:,1))) (z(tri(:,2)) - z(tri(:,1)))];  % P2 - P1
    W1 = [(x(tri(:,3)) - x(tri(:,1))) (y(tri(:,3)) - y(tri(:,1))) (z(tri(:,3)) - z(tri(:,1)))];  % P3 - P1;
    mV1 = sqrt(sum(V1.^2,2));
    mW1 = sqrt(sum(W1.^2,2));
    
    V2 = [(x(tri(:,3)) - x(tri(:,2))) (y(tri(:,3)) - y(tri(:,2))) (z(tri(:,3)) - z(tri(:,2)))];  % P3 - P2
    W2 = [(x(tri(:,1)) - x(tri(:,2))) (y(tri(:,1)) - y(tri(:,2))) (z(tri(:,1)) - z(tri(:,2)))];  % P1 - P2;
    mV2 = sqrt(sum(V2.^2,2));
    mW2 = sqrt(sum(W2.^2,2));
    
    V3 = [(x(tri(:,1)) - x(tri(:,3))) (y(tri(:,1)) - y(tri(:,3))) (z(tri(:,1)) - z(tri(:,3)))];  % P1 - P3
    W3 = [(x(tri(:,2)) - x(tri(:,3))) (y(tri(:,2)) - y(tri(:,3))) (z(tri(:,2)) - z(tri(:,3)))];  % P2 - P3;
    mV3 = sqrt(sum(V3.^2,2));
    mW3 = sqrt(sum(W3.^2,2));
    
    cos1 = ((V1(:,1).*W1(:,1))+(V1(:,2).*W1(:,2))+(V1(:,3).*W1(:,3)))./(mV1.*mW1);
    cos2 = ((V2(:,1).*W2(:,1))+(V2(:,2).*W2(:,2))+(V2(:,3).*W2(:,3)))./(mV2.*mW2);
    cos3 = ((V3(:,1).*W3(:,1))+(V3(:,2).*W3(:,2))+(V3(:,3).*W3(:,3)))./(mV3.*mW3);
    
    cosAngle = [cos1 cos2 cos3];
    [~,j] = min(abs(cosAngle),[],2);
    k = (1:numTri)';
    k = sub2ind([numTri 3],k,j);
    
    Vx = [V1(:,1) V2(:,1) V3(:,1)];
    Vy = [V1(:,2) V2(:,2) V3(:,2)];
    Vz = [V1(:,3) V2(:,3) V3(:,3)];
    Wx = [W1(:,1) W2(:,1) W3(:,1)];
    Wy = [W1(:,2) W2(:,2) W3(:,2)];
    Wz = [W1(:,3) W2(:,3) W3(:,3)];
    
    % N is the surface normal, then:
    
    Nx =(Vy(k).*Wz(k))-(Vz(k).*Wy(k));
    Ny =(Vz(k).*Wx(k))-(Vx(k).*Wz(k));
    Nz =(Vx(k).*Wy(k))-(Vy(k).*Wx(k));
    
    module = sqrt((Nx.^2)+(Ny.^2)+(Nz.^2));
    
    x_centroid = mean([x(tri(:,1)) x(tri(:,2)) x(tri(:,3))],2);
    y_centroid = mean([y(tri(:,1)) y(tri(:,2)) y(tri(:,3))],2);
    z_centroid = mean([z(tri(:,1)) z(tri(:,2)) z(tri(:,3))],2);
    
%     maxDist = 0;
%     for j=1:numTri
%         [intersectionPoint, distFromOrigin, intersectedFaces] = F_intersectMultipleLineMesh([x_centroid(j) y_centroid(j) z_centroid(j)],[Nx(j) Ny(j) Nz(j)],[Nodes(tri(j,1),:);Nodes(tri(j,2),:);Nodes(tri(j,3),:)],[1 2 3]);
%         x_centroid(j) = intersectionPoint(1);
%         y_centroid(j) = intersectionPoint(2);
%         z_centroid(j) = intersectionPoint(3);
%         maxDist = max(maxDist,abs(distFromOrigin))
%     end
    
    if cellArray
        nTri{i} = [(Nx./module) (Ny./module) (Nz./module)]; % Normal vectors
        bTri{i} = [x_centroid y_centroid z_centroid]; % Normal positions
    else
        nTri = [(Nx./module) (Ny./module) (Nz./module)]; % Normal vectors
        bTri = [x_centroid y_centroid z_centroid]; % Normal positions
    end
end

