function [iNodes,nNodes] = F_getNodeNormals(Nodes,iSurfaces,bTri,nTri)
%[iNodes,nNodes] = F_getNodeNormals(Nodes,iSurfaces,bTri,nTri)
%   The function computes the normals to all nodes. "Nodes" is a nx3
%   array containing the n vertices of the mesh (with no redundancies).
%   "iSurfaces" is a cell array of 3xnT uint32 arrays containing the
%   indexed triangles of the mesh surfaces. Every cell contains one surface. 
%   "bTri" and "nTri" are two optional input parameters.
%   They are respectively the baricentres of the triangles (origins of the 
%   triangles' normals) and the triangles' normals. They are calculated
%   internally to the function, if not provided as inputs.
%   "iNodes" and "nNodes" are respectively the index of all nodes (divided 
%   by cells, depenting on the surface they belong to) and the the nodes'
%   normals (also divided into cells). Nodes(iNodes,:) provides the
%   coordinates of the nodes, were the normals nNodes are to be plotted.
%
%   Author:         University of Strathclyde
%                   Centre for Ultrasonic Engineering
%                   Dr Carmelo Mineo
%
%   Last update: 06/07/2016


cellArray = 0;

if iscell(iSurfaces)
    cellArray = 1;
    ns = numel(iSurfaces);
    iNodes = cell(1,ns);
    nNodes = cell(1,ns);
else
    ns = 1;
    iNodes = [];
    nNodes = [];
end

if nargin<3
    [bTri,nTri] = F_getTriangleNormals(Nodes,iSurfaces);
end

for i=1:ns
    if cellArray
        tri = iSurfaces{i};
        N = nTri{i};
        PN = bTri{i};
    else
        tri = iSurfaces;
        N = nTri;
        PN = bTri;
    end
    
    [iNode,~,ic] = unique(tri);
    n_points = length(iNode);
    
    ic = reshape(ic,size(tri));
    connectedTri = uint32(zeros(n_points,100));
    occurrence = uint8(zeros(n_points,1));
    for j=1:n_points
        [indTri,~] = find(ic==j);
        occurrence(j) = length(indTri);
        connectedTri(j,1:occurrence(j)) = indTri;
    end
    
    connectedTri(:,(max(occurrence)+1):end) = [];
    
    x = Nodes(iNode,1);
    y = Nodes(iNode,2);
    z = Nodes(iNode,3);
    
    sct = size(connectedTri);
    dist = inf(sct);
    for j=1:sct(2)
        dist(connectedTri(:,j)~=0,j) = sqrt(((x(connectedTri(:,j)~=0)-PN(connectedTri(connectedTri(:,j)~=0,j),1)).^2) + ((y(connectedTri(:,j)~=0)-PN(connectedTri(connectedTri(:,j)~=0,j),2)).^2) + ((z(connectedTri(:,j)~=0)-PN(connectedTri(connectedTri(:,j)~=0,j),3)).^2));
    end
    
    nNode = zeros(n_points,3);
    connectedTri(connectedTri==0) = 1;
    
    for j=1:sct(2)
        rDist = (dist(:,j).^(-1));
        nNode = nNode + [(N(connectedTri(:,j),1).*rDist) (N(connectedTri(:,j),2).*rDist) (N(connectedTri(:,j),3).*rDist)];
    end
    
    module = sqrt((nNode(:,1).^2)+(nNode(:,2).^2)+(nNode(:,3).^2));
    nNode = [(nNode(:,1)./module) (nNode(:,2)./module) (nNode(:,3)./module)];
    
    if cellArray
        nNodes{i} = nNode;
        iNodes{i} = iNode; % Normal positions
    else
        nNodes = nNode;
        iNodes = iNode; % Normal positions
    end
end

