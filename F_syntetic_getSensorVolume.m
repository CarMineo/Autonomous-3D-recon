function [patches,edges] = F_syntetic_getSensorVolume(sensor)
%[patches,edges] = F_syntetic_getSensorVolume(sensor)
%Computes patches and edges to visualize sensor detection volume
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
%
%   Outputs:
%       patches - surfaces plottable with matlab functions that support
%                 patches (e.g. fill3)
%           patches.lower - plane at the bottom of the detection volume;
%           patches.upper - plane at the bottom of the detection volume;
%           patches.side1 - 1st lateral face;
%           patches.side2 = 2nd lateral face;
%           patches.side3 - 3rd lateral face;
%           patches.side4 = 4th lateral face;
%       edges - edges plottable with matlab functions that support lines
%               (e.g. plot3)
%           edges.lower - perimeter of plane at the bottom of the detection volume;
%           patches.upper - perimeter of plane at the top of the detection volume;
%           edges.side1 - perimeter of 1st lateral face;
%           edges.side2 - perimeter of 2nd lateral face;
%           edges.side3 - perimeter of 3rd lateral face;
%           edges.side4 - perimeter of 4th lateral face;
%           edges.originSide1 - perimeter of triangular face linking 1st
%                               edge of upper face with sensor origin; 
%           edges.originSide2 - perimeter of triangular face linking 2nd
%                               edge of upper face with sensor origin; 
%           edges.originSide3 - perimeter of triangular face linking 3rd
%                               edge of upper face with sensor origin;
%           edges.originSide4 - perimeter of triangular face linking 4th
%                               edge of upper face with sensor origin;
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

patches = [];
edges = [];

positionRGBD = sensor.position;
R = sensor.rotationMatrix;

if strcmp(sensor.type,'cartesian')
    azRange = sensor.azRange;
    elRange = sensor.elRange;
    
    ux = tan(azRange);
    uy = tan(elRange);
    [uX,uY] = meshgrid(ux,uy);
    uZ = ones(size(uX));
    
    lines = [uX(:) uY(:) uZ(:)]/R;
    lines(3:4,:) = flipud(lines(3:4,:));
    % normLines = sqrt(sum(lines.^2,2));
    % lines = lines./normLines;
    
    lowerLimitPlane = lines.*(sensor.range(1));
    lowerLimitPlane(:,1) = lowerLimitPlane(:,1) + positionRGBD(1);
    lowerLimitPlane(:,2) = lowerLimitPlane(:,2) + positionRGBD(2);
    lowerLimitPlane(:,3) = lowerLimitPlane(:,3) + positionRGBD(3);
    
    upperLimitPlane = lines.*(sensor.optimumDist);
    upperLimitPlane(:,1) = upperLimitPlane(:,1) + positionRGBD(1);
    upperLimitPlane(:,2) = upperLimitPlane(:,2) + positionRGBD(2);
    upperLimitPlane(:,3) = upperLimitPlane(:,3) + positionRGBD(3);
    
    patches.lower = lowerLimitPlane;
    patches.upper = upperLimitPlane;
    patches.side1 = [lowerLimitPlane(1,:);
        upperLimitPlane(1,:);
        upperLimitPlane(2,:);
        lowerLimitPlane(2,:)];
    patches.side2 = [lowerLimitPlane(2,:);
        upperLimitPlane(2,:);
        upperLimitPlane(3,:);
        lowerLimitPlane(3,:)];
    patches.side3 = [lowerLimitPlane(3,:);
        upperLimitPlane(3,:);
        upperLimitPlane(4,:);
        lowerLimitPlane(4,:)];
    patches.side4 = [lowerLimitPlane(4,:);
        upperLimitPlane(4,:);
        upperLimitPlane(1,:);
        lowerLimitPlane(1,:)];
    
    edges.lower = [patches.lower;
        patches.lower(1,:)];
    edges.upper = [patches.upper;
        patches.upper(1,:)];
    edges.side1 = [patches.side1(1,:);
        patches.side1(2,:)];
    edges.side2 = [patches.side2(1,:);
        patches.side2(2,:)];
    edges.side3 = [patches.side3(1,:);
        patches.side3(2,:)];
    edges.side4 = [patches.side4(1,:);
        patches.side4(2,:)];
    edges.originSide1 = [positionRGBD(1,:);
        patches.lower(1,:)];
    edges.originSide2 = [positionRGBD(1,:);
        patches.lower(2,:)];
    edges.originSide3 = [positionRGBD(1,:);
        patches.lower(3,:)];
    edges.originSide4 = [positionRGBD(1,:);
        patches.lower(4,:)];
else
    elMin = sensor.elMin;
    elMax = sensor.elMax;
    azMin = sensor.azMin;
    azMax = sensor.azMax;
    
    % TO BE COMPLETED
end

% figure(10)
% fill3(patches.lower(:,1),patches.lower(:,2),patches.lower(:,3),[0.8 0.8 0.8],'linestyle','none','facealpha',0.5); hold on;
% fill3(patches.upper(:,1),patches.upper(:,2),patches.upper(:,3),[0.8 0.8 0.8],'linestyle','none','facealpha',0.5); hold on;
% fill3(patches.side1(:,1),patches.side1(:,2),patches.side1(:,3),[0.8 0.8 0.8],'linestyle','none','facealpha',0.5); hold on;
% fill3(patches.side2(:,1),patches.side2(:,2),patches.side2(:,3),[0.8 0.8 0.8],'linestyle','none','facealpha',0.5); hold on;
% fill3(patches.side3(:,1),patches.side3(:,2),patches.side3(:,3),[0.8 0.8 0.8],'linestyle','none','facealpha',0.5); hold on;
% fill3(patches.side4(:,1),patches.side4(:,2),patches.side4(:,3),[0.8 0.8 0.8],'linestyle','none','facealpha',0.5); hold on;
% plot3(edges.lower(:,1),edges.lower(:,2),edges.lower(:,3),'color',[0 0 0],'linewidth',1); hold on;
% plot3(edges.upper(:,1),edges.upper(:,2),edges.upper(:,3),'color',[0 0 0],'linewidth',1); hold on;
% plot3(edges.side1(:,1),edges.side1(:,2),edges.side1(:,3),'color',[0 0 0],'linewidth',1); hold on;
% plot3(edges.side2(:,1),edges.side2(:,2),edges.side2(:,3),'color',[0 0 0],'linewidth',1); hold on;
% plot3(edges.side3(:,1),edges.side3(:,2),edges.side3(:,3),'color',[0 0 0],'linewidth',1); hold on;
% plot3(edges.side4(:,1),edges.side4(:,2),edges.side4(:,3),'color',[0 0 0],'linewidth',1); hold on;
% grid on

%------------- END CODE --------------

end

