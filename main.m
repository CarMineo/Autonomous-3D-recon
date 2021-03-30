close all; clc; clear;

%This matlab script demonstrates the execution of the autonomous 3D
%reconstruction pipeline, relative to the paper titled: "Autonomous 3D 
%geometry reconstruction through robot-manipulated optical sensors", by 
%C. Mineo, D. Cerniglia, V. Ricotta and B. Reitinger.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                       USER-DEFINED INPUTS                           %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input and output subfolders
inputDir = 'inputs';
outputDir = 'outputs';

% Sensor parameters
sensor = [];
sensor.position =       [435 435 350]; % Initial pose Cartesian coordinates
sensor.rotationMatrix = [0  1  0;      % Initial pose rotation matrix
                         1  0  0;
                         0  0 -1];

sensor.resAz = 640;                    % Sensor azimutal resolution
sensor.resEl = 480;                    % Sensor elavation resolution
sensor.azRange = [-1.2909/2 1.2909/2]; % Azimutal angle range
sensor.elRange = [-1.0816/2 1.0816/2]; % Elevation angle range
sensor.range = [0 600];             % Sensor depth range [min max]
sensor.optimumDist_accuracy = 200;	% Sensor accuracy-dependant stand-off
sensor.optimumDist_sampling = 200;	% Sensor sampling-dependant stand-off
sensor.optimumDist = 200;         	% Sensor optimum stand-off
sensor.type = 'cartesian';        	% Sensor type

% Data processing parameters
desRes = 0.05;               % Target sampling density (points/mm^2)
safetyDist = 50;             % Safety distance between sensor pose and sample
processingMode = 'vertices'; % Processing method ('vertices' or 'faces')
dsMode = 'best';             % Downsampling mode ('best', 'worst' or 'average')
nAngTests = 5;               % Number of sensor orientations to consider

% Robot data
robotData = [];     % robotData contains information about the sensor
                    % manipulator. If it is empty (robotData=[]), no
                    % kinematic constraint is considered. If pose
                    % reachbility must be checked, robotData must contain
                    % the following information:
                    % robotData.Links: [6×1 double] (length of robot joints)
                    % robotData.Joints: [6×3 double] (centre of kiematic joints)
                    % robotData.robotPrefConf: (robot preferred configuration.
                    %                          If 0 best configuration is
                    %                          automatically determined)
                    % robotData.toolParameters: [0 0 0 0 0 0] (tool parameters)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                     PRELIMINAR CALCULATIONS                         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load sample and support geometry
sampleTriangulation = stlread([inputDir '\sample.stl']);

% Define full directory path for input and output folders. The '/'
% separator is preferred for better cross-platform compatibility
inputDir = [strrep(pwd,'\','/') '/' inputDir];
outputDir = [strrep(pwd,'\','/') '/' outputDir];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%               PREPARATION OF PLOTTING ENVIRONEMENT                  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screenSize = get(0, 'ScreenSize');
hf = figure(1); set(hf, 'Position', get(0, 'Screensize'));

% Subplot for pose simulation
hSubplot1 = subplot(1,2,1);
hSample = patch('Faces',sampleTriangulation.ConnectivityList,...
                'Vertices',sampleTriangulation.Points,...
                'edgecolor','none','facecolor',[0.4 0.6 1],...
                'facealpha',1,'DiffuseStrength',0.8);
material(hSample,'dull');
hz=zoom; setAxes3DPanAndZoomStyle(hz,hSubplot1,'camera');
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
zlabel('z','FontSize',16,'Rotation',0);
axis image;grid on; hold on;
view(60,30);light; lighting gouraud;

% Subplot for 3D reconstruction result
hSubplot2 = subplot(1,2,2);
hMesh = trisurf([nan nan nan],nan,nan,nan,0,'FaceColor','interp',...
                'EdgeColor',[0.25 0.25 0.25],'facealpha',0.75,...
                'edgealpha',0.25);
hTitle = title('Visited poses: 0  -  Compl. estimate: 0%','fontsize',14,...
               'fontweight','bold');

% Custom colormap definition
mymap = [linspace(1,1,128)'   linspace(0.3,1,128)'   linspace(0.3,0,128)';
         linspace(1,0,128)'   linspace(1,0.8,128)'   zeros(128,1)];

colormap(hSubplot2,mymap);

% Sampling density expected when using the given sensor optimum distance
rho = 0.5*((sensor.resAz*sensor.resEl)/(4*tan(sensor.azRange(2))*tan(sensor.elRange(2))*(sensor.optimumDist^2)));

% Setting the color limits
set(hSubplot2,'CLim',[0 rho]);

% Plot colorbar
hc = colorbar('southoutside','FontSize',10);
hc.Label.String = 'Corrected cumulative sampling density (\lambda)';
hc.Label.FontSize = 12;
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
zlabel('z','FontSize',16,'Rotation',0);
axis image; grid on;
view(60,30);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%           INITIALIZATION OF VARIABLES FOR ITERATIVE LOOP            %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize files for data storing and memory mapping
[rawWriteID,fDetWriteID,iDsData] = F_initSaveAndDownsample(outputDir);

% Compute Cartesian lower and upper bounts (lb, ub) for test poses
% Lower bound
lb = [min(sampleTriangulation.Points(:,1))-sensor.optimumDist,...   % min X
      min(sampleTriangulation.Points(:,2))-sensor.optimumDist,...   % min Y
      60,...                                                        % min Z
      -pi,...                                                       % min A
      -pi,...                                                       % min B
      -pi];                                                         % min C

%Upper bound
ub = [max(sampleTriangulation.Points(:,1))+sensor.optimumDist,...   % max X
      max(sampleTriangulation.Points(:,2))+sensor.optimumDist,...   % max Y
      max(sampleTriangulation.Points(:,3))+sensor.optimumDist,...   % max Z
      pi,...                                                        % max A
      pi,...                                                        % max B
      pi];                                                          % max C

dsCubeSide = sqrt(1/(sqrt(2)*desRes));	% Downsampling cube edge size

% Graphical appearance of sensor visualization
hSensor = [];
handles = [];
hxv = [];
hyv = [];
hzv = [];


% Variable to store all sensor poses
allSensors = [];

% Pose counter
k = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%           INCREMENTAL DATA PROCESSING AND 3D RECONSTRUCTION         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while 1
    k=k+1;
    
    % Update visualization of sensor position
    delete(hSensor);
    pose = rigid3d(sensor.rotationMatrix',...
                   [sensor.position(1)+(-sensor.rotationMatrix(1,3)*30) ...
                    sensor.position(2)+(-sensor.rotationMatrix(2,3)*30) ...
                    sensor.position(3)+(-sensor.rotationMatrix(3,3)*30)]);
    hSensor = plotCamera('Parent',hSubplot1,'AbsolutePose', pose,...
                         'Opacity', 0.3,'Size',20,'color',[0.4 0.4 0.4]);
    
    [patches,edges] = F_syntetic_getSensorVolume(sensor);
    handles = F_syntetic_plotSensorVolume(hSubplot1,handles,patches,edges);
    
    % Update visualization of sensor reference system arrows
    delete(hxv);
    delete(hyv);
    delete(hzv);
    
    P1 = sensor.position;
    vector = sensor.rotationMatrix(:,1)';
    P2 = P1 + vector.*100;
    hxv = F_3Darrow(P1,P2,'color',[0.9 0.0 0.0],'stemWidth',4,...
                    'tipWidth',10,'Parent',hSubplot1);
    
    vector = sensor.rotationMatrix(:,2)';
    P2 = P1 + vector.*100;
    hyv = F_3Darrow(P1,P2,'color',[0.0 0.7 0.0],'stemWidth',4,...
                    'tipWidth',10,'Parent',hSubplot1);
    
    vector = sensor.rotationMatrix(:,3)';
    P2 = P1 + vector.*100;
    hzv = F_3Darrow(P1,P2,'color',[0.0 0.0 1],'stemWidth',4,...
                    'tipWidth',10,'Parent',hSubplot1);
    drawnow;
    
    % Simulate acquisition of point cloud from sensor
    [newData,intersectedFaces] = F_syntetic_receiveCloudFromRGBD(sensor,...
          sampleTriangulation.Points,sampleTriangulation.ConnectivityList);

    % Append current sensor to the list of sensors used in previous poses
    allSensors{k} = sensor;
    mPData = [];
    [rawWriteID,fDetWriteID,mRData,mPData,iRData,iPData,iDsData,...
        iClusters] = F_saveAndDownsample(rawWriteID,fDetWriteID,newData,...
        iDsData,sensor,dsCubeSide,dsMode,outputDir);
    
    % Generate mesh through Poisson reconstruction algorithm
    mesh = [];
    [mesh.vertices,mesh.faces] = F_poissonReconstruction(outputDir,...
                                 'dsData.ply',outputDir,'reconShape.ply');
    
    % Compute mesh normals and mesh area
    [mesh.facesCentres,mesh.facesNormals] = F_getTriangleNormals(mesh.vertices,mesh.faces);
    [~,mesh.verticesNormals] = F_getNodeNormals(mesh.vertices,...
                               mesh.faces,mesh.facesCentres,mesh.facesNormals);
    [mesh.facesArea,mesh.verticesArea,mesh.totalArea] = F_meshArea(mesh.faces,mesh.vertices);
    
    
    % Compute sampling densities and centrality factors
    [achievedSampling,achievedDensity,achievedCentrality...
        ] = F_currentDensity(allSensors,mesh,processingMode);
    
    % Update visualization of reconstructed geometry
    if strcmp(processingMode,'vertices')
        vertexAlpha = ones(size(achievedDensity)).*0.75;
        set(hMesh,'FaceAlpha','interp');
        set(hMesh,'AlphaDataMapping','none');
        set(hMesh,'Faces',mesh.faces,'Vertices',mesh.vertices,'CData',...
            achievedDensity.*achievedCentrality,'FaceVertexAlphaData',vertexAlpha);
    else
        hMesh = trisurf([nan nan nan],nan,nan,nan,0,'EdgeColor','none');
        set(hMesh,'FaceVertexAlphaData',1);
        set(hMesh,'Faces',mesh.faces,'Vertices',mesh.vertices);
        set(hMesh,'CData',achievedDensity.*achievedCentrality);
    end
    
    % Compute initial test points for probing the objective function
    initialTestLocations = F_initialTestLocations(sensor.optimumDist,...
        mesh,achievedSampling,desRes,nAngTests,20,lb,ub,robotData,processingMode);
    
    % Compute completion estimate and number of visitable test poses
    expectedPoints = mesh.totalArea*desRes;
    saturatedAchievedSampling = achievedSampling;
    samplingTargetReached = saturatedAchievedSampling>desRes;
    saturatedAchievedSampling(samplingTargetReached) = desRes;
    
    sampledPoints = sum(mesh.verticesArea.*saturatedAchievedSampling);
    completionEstimate = (sampledPoints/expectedPoints)*100;
    visitablePoses = size(initialTestLocations,1);
    
    % Print number of visited poses and completion estimate
    set(hTitle,'String',['Visited poses: ' num2str(k) '  -  Compl. estimate: ' num2str(completionEstimate,'%.0f') '%']);
    
    % Save current figure as .fig and .tiff file
    savefig(hf,[outputDir '\Pose ' num2str(k) '.fig']);
    print([outputDir '\Pose ' num2str(k) '.tiff'],'-dtiff','-r600');
    
    if (visitablePoses>0) && (completionEstimate<99.5) % Verify stopping criteria
        iLowSampling = find(~samplingTargetReached);
        
        % Objective function
        fun = @(x) F_predictDensity([x(1) x(2) x(3) x(4) x(5) x(6)],...
                                    sensor,mesh,achievedSampling,...
                                    achievedCentrality,iLowSampling,...
                                    desRes,robotData,safetyDist,...
                                    processingMode);
        
        % Probe the search space and find best next position among test poses                       
        options = optimoptions('surrogateopt','InitialPoints',initialTestLocations,...
                               'Display','off','MaxFunctionEvaluations',100,...
                               'PlotFcn','surrogateoptplot','UseParallel',false);
        
        [x,fval,exitflag,osur] = surrogateopt(fun,lb,ub,options);
        
        % Update sensor pose
        sensor.position = [x(1) x(2) x(3)];
        sensor.rotationMatrix = F_eul2rotm([x(4) x(5) x(6)],'rad');
    else
        % Break the incremental reconstruction if stopping criteria are met
        break; 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                      CLOSE DATA STORING FILES                       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F_closeSaveAndDownsample(rawWriteID,fDetWriteID);