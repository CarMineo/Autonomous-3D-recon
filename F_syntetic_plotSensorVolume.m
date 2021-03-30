function handles = F_syntetic_plotSensorVolume(axisHandle,handles,patches,edges)
%handles = F_syntetic_plotSensorVolume(axisHandle,handles,patches,edges)
%Plots patches and edges (computed by F_syntetic_getSensorVolume)
%
%   Inputs:
%       axisHandle - handle of parent axes
%       handles - handles of existing plotted entities (empty if no entities exist)
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
%   Outputs:
%          handles - handles of plotted entities
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

faceColor = [0.5 0.5 0.5];
edgeColor = [0.5 0.5 0.5];
faceAlpha = 0.3;
edgeWidth = 1.5;

if ~isempty(handles)
    if ~isempty(handles.patches)
        graphics = isgraphics(handles.patches);
        delete(handles.patches(graphics==1));
    end
    if ~isempty(handles.edges)
        graphics = isgraphics(handles.edges);
        delete(handles.edges(graphics==1));
    end
    drawnow
end

handles = [];

if ~isempty(patches) && ~isempty(edges)
    handles.patches = [];
    handles.edges = [];
    
    H = gca;
    axes(axisHandle);
    hold on;
    %handles.patches(1) = fill3(patches.lower(:,1),patches.lower(:,2),patches.lower(:,3),faceColor,'linestyle','none','facealpha',faceAlpha); hold on;
%     handles.patches(2) = fill3(patches.upper(:,1),patches.upper(:,2),patches.upper(:,3),faceColor,'linestyle','none','facealpha',faceAlpha); hold on;
    handles.patches(3) = fill3(patches.side1(:,1),patches.side1(:,2),patches.side1(:,3),faceColor,'linestyle','none','facealpha',faceAlpha); hold on;
    handles.patches(4) = fill3(patches.side2(:,1),patches.side2(:,2),patches.side2(:,3),faceColor,'linestyle','none','facealpha',faceAlpha); hold on;
    handles.patches(5) = fill3(patches.side3(:,1),patches.side3(:,2),patches.side3(:,3),faceColor,'linestyle','none','facealpha',faceAlpha); hold on;
    handles.patches(6) = fill3(patches.side4(:,1),patches.side4(:,2),patches.side4(:,3),faceColor,'linestyle','none','facealpha',faceAlpha); hold on;
%    handles.edges(1) = plot3(edges.lower(:,1),edges.lower(:,2),edges.lower(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
    handles.edges(2) = plot3(edges.upper(:,1),edges.upper(:,2),edges.upper(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
    handles.edges(3) = plot3(edges.side1(:,1),edges.side1(:,2),edges.side1(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
    handles.edges(4) = plot3(edges.side2(:,1),edges.side2(:,2),edges.side2(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
    handles.edges(5) = plot3(edges.side3(:,1),edges.side3(:,2),edges.side3(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
    handles.edges(6) = plot3(edges.side4(:,1),edges.side4(:,2),edges.side4(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
%     
%     handles.edges(7) = plot3(edges.originSide1(:,1),edges.originSide1(:,2),edges.originSide1(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
%     handles.edges(8) = plot3(edges.originSide2(:,1),edges.originSide2(:,2),edges.originSide2(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
%     handles.edges(9) = plot3(edges.originSide3(:,1),edges.originSide3(:,2),edges.originSide3(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
%     handles.edges(10) = plot3(edges.originSide4(:,1),edges.originSide4(:,2),edges.originSide4(:,3),'color',edgeColor,'linewidth',edgeWidth); hold on;
    axes(H);
end

%------------- END CODE --------------

end

