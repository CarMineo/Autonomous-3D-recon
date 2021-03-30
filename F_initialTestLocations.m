function initialTestPoses = F_initialTestLocations(optimumDist,mesh,corrSamDensity,targetDensity,nAngTests,nPoints,lb,ub,robotData,mode)
%initialTestLocations = F_initialTestLocations(bestDepth,mesh,corrSamDensity,targetDensity,nAngTests,nPoints,lb,ub,robotData,mode)
%Finds initial test poses suitable to probe the search space.
%
%   Inputs:
%       optimumDist - optimum sensor stand off
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
%       targetDensity - target sampling density
%       nAngTests - number of test orientations to consider for each point
%       nPoints - number of points to 
%       lb - lower Cartesian bounds
%       ub - upper Cartesian bounds
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
%       mode - Processing method ('vertices' or 'faces')
%
%   Outputs:
%       initialTestPoses - [(nAngTests x nPoints) x 6]. Initial test
%                          postitions
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
    meshPoints = mesh.facesCentres;
    meshNormals = mesh.facesNormals;
elseif strcmp(mode,'vertices')
    meshPoints = mesh.vertices;
    meshNormals = mesh.verticesNormals;
end

% Find points whose sampling factor is lower than threshold
i = find(corrSamDensity<targetDensity);

if ~isempty(i)
    [corrSamDensity,I] = sort(corrSamDensity(i,:),'ascend');
    meshPoints = meshPoints(i(I),:);
    meshNormals = meshNormals(i(I),:);
    
    offsets = meshNormals.*optimumDist;
    initialTestPoses = [(meshPoints(:,1) + offsets(:,1)) (meshPoints(:,2) + offsets(:,2)) (meshPoints(:,3) + offsets(:,3)) meshNormals];
    
    if ~isempty(lb) && ~isempty(ub)
        % Remove out-of-bounding-box points
        j = find((initialTestPoses(:,1)>lb(1)) & (initialTestPoses(:,1)<ub(1)) &...
            (initialTestPoses(:,2)>lb(2)) & (initialTestPoses(:,2)<ub(2)) &...
            (initialTestPoses(:,3)>lb(3)) & (initialTestPoses(:,3)<ub(3)));
    else
        j = 1:size(initialTestPoses,1);
    end
    
    if ~isempty(j)
        corrSamDensity = corrSamDensity(j,:);
        meshPoints = meshPoints(j,:);
        meshNormals = meshNormals(j,:);
        offsets = offsets(j,:);
        initialTestPoses = initialTestPoses(j,:);
        
        [~,~,iBins] = histcounts(corrSamDensity,100);
        if nPoints>size(meshNormals,1)
            nPoints = size(meshNormals,1);
        end
        
        % Rearrange points according to maximum distances
        targetPoints = zeros(nPoints,6);
        binNumber = 0;
        counter = 0;
        while (counter<nPoints) && (binNumber<100)
            binNumber = binNumber + 1;
            nPointsInBin = length(find(iBins==binNumber));
            if nPointsInBin>0
                sortedPoints = initialTestPoses(iBins==binNumber,:);
                counter = counter + 1;
                targetPoints(counter,:) = sortedPoints(1,:);
                
                binCounter = 1;
                dist = inf(nPointsInBin,1);
                validBinIndices = true(nPointsInBin,1);
                p2 = sortedPoints(:,4:6);
                while (counter<nPoints) && (binCounter < nPointsInBin)
                    dists = sqrt(((sortedPoints(validBinIndices,1)-targetPoints(counter,1)).^2)+...
                        ((sortedPoints(validBinIndices,2)-targetPoints(counter,2)).^2)+...
                        ((sortedPoints(validBinIndices,3)-targetPoints(counter,3)).^2));
                    p1 = repmat(targetPoints(counter,4:6),nPointsInBin,1);
                    crossProd = cross(p1,p2(validBinIndices,:));
                    normCrossProd = sqrt(sum(crossProd.^2,2));
                    angle = abs(atan2(normCrossProd, sum(p1.*p2(validBinIndices,:),2)));
                    
                    dist(validBinIndices) = min([dist(validBinIndices) dists],[],2);
                    
                    i = find(dist(validBinIndices)==max(dist(validBinIndices)),1);
                    iValid = find(validBinIndices);
                    
                    % Check if line of sight exists
                    [~, distFromOrigin, ~] = F_intersectMultipleLineMesh(sortedPoints(iValid(i),1:3),-sortedPoints(iValid(i),4:6).*optimumDist,vertices,faces);
                    
                    if (distFromOrigin >= (1-(1000*eps('single'))))
                        
                        % Check if the pose is reachable by robot   
                        if ~isempty(robotData)
                            usedConf = 1;
                            t = linspace(0,pi,nAngTests + 1); %angle
                            t = t(1:nAngTests);
                            count = 0;
                            while (usedConf>0) && (count<nAngTests)
                                count = count + 1;
                                robotPose = F_getRobotPose(sortedPoints(iValid(i),1:3),-sortedPoints(iValid(i),4:6),t(count),robotData);
                                rotM = F_eul2rotm(robotPose(4:6),'deg');
                                robotPose(1,1:3) = robotPose(1,1:3) + (rotM(:,2)').*0;
                                robotPose(1,1:3) = robotPose(1,1:3) + (rotM(:,1)').*(-17.5);
                                robotPose(1,1:3) = robotPose(1,1:3) + (rotM(:,3)').*(-33.2);
                                
                                [~,usedConf] = F_getConfSpaceTargets(robotPose,robotData);
                            end
                        else
                            usedConf = 1;
                        end
                            
                        
                        if usedConf>0 && usedConf<7
                            counter = counter + 1;
                            binCounter = binCounter + 1;
                            targetPoints(counter,:) = sortedPoints(iValid(i),:);
                        else
                            validBinIndices(iValid(i)) = false;
                            dist(iValid(i)) = 0;
                            nPointsInBin = nPointsInBin - 1;
                        end
                    else
                        validBinIndices(iValid(i)) = false;
                        dist(iValid(i)) = 0;
                        nPointsInBin = nPointsInBin - 1;
                    end
                end
            end
        end
        
        if nPoints>counter
            nPoints = counter;
        end
        
        if nPoints>0
            t = linspace(0,pi,nAngTests + 1); %angle
            t = t(1:nAngTests);
            x = cos(t);
            y = sin(t);
            z = 0;
            
            %compute rotate theta and axis
            zaxis = [0 0 1];
            
            initialTestPoses = zeros(nPoints*nAngTests,6);
            for i=1:nPoints
                initialTestPoses((((i-1)*nAngTests)+1):(i*nAngTests),1:3) = repmat(targetPoints(i,1:3),nAngTests,1);
                normal = -targetPoints(i,4:6);
                for j=1:nAngTests
                    if isequal(normal,zaxis)
                        fx = x(j);
                        fy = y(j);
                        fz = z;
                    elseif isequal(-normal,zaxis)
                        fx = -x(j);
                        fy = -y(j);
                        fz = -z;
                    else
                        normal = normal/norm(normal);
                        ang = acos(dot(zaxis,normal));
                        axis = cross(zaxis, normal)/norm(cross(zaxis, normal));
                        % A skew symmetric representation of the normalized axis
                        axis_skewed = [ 0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0];
                        % Rodrigues formula for the rotation matrix
                        R = eye(3) + sin(ang)*axis_skewed + (1-cos(ang))*axis_skewed*axis_skewed;
                        fx = R(1,1)*x(j) + R(1,2)*y(j) + R(1,3)*z;
                        fy = R(2,1)*x(j) + R(2,2)*y(j) + R(2,3)*z;
                        fz = R(3,1)*x(j) + R(3,2)*y(j) + R(3,3)*z;
                    end
                    
                    u = [fx fy fz];
                    v = cross(normal,u);
                    R = [u' v' normal'];
                    
                    initialTestPoses(((i-1)*nAngTests)+j,4:6) = F_rotm2eul(R,'rad');
                end
            end
        else
            initialTestPoses = [];
        end
    end
end

%------------- END CODE --------------

end

