function [vertices,faces] = F_poissonReconstruction(inputDir,inputPLYname,outputDir,outputPLYname)
%[meshVertices,meshFaces] = F_poissonReconstruction(inputDir,inputPLYname,outputDir,outputPLYname)
%This function operates the Poisson mesh reconstruction of a point cloud 
%stored in a PLY file. 
%
%   Inputs:
%       inputDir - string containing the path of the input directory
%       inputPLYname - string containing the name of the PLY file storing
%                      the oriented point cloud
%       outputDir - string containing the path of the output directory
%       outputPLYname - string containing the name of the PLY file storing
%                       the output mesh
%
%   Outputs:
%       faces - [nx3] indices of vertices of mesh faces
%       vertices - [mx3] vertices of mesh
%
%   Dependencies:
%       The function needs to find 'PoissonRecon.exe' and
%       'SurfaceTrimmer.exe' in the current matlab working directory.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b
%              PoissonRecon.exe and SurfaceTrimmer.exe from 
%              Adaptive Multigrid Solvers (Version 13.72)
%              (https://github.com/mkazhdan/PoissonRecon)


%------------- BEGIN CODE --------------

w = 0.1;  % Poisson reconstruction minimum width

path = mfilename('fullpath');
path = strsplit(path, 'F_poissonReconstruction');
pathDir = path{1};

hw = waitbar(0,'Poisson mesh generation. Please wait...');
waitbar(0.3,hw);
currentFolder = pwd;
cd(pathDir);

copyfile([inputDir '/' inputPLYname],inputPLYname,'f');

% system(['PoissonRecon --in dsData.ply --out poisReconMesh.ply --width ' num2str(gridSpacing) ' --verbose --density']);
% system('SurfaceTrimmer --in poisReconMesh.ply --out poisReconMesh.ply --trim 6.5 --smooth 0 --verbose');
system(['PoissonRecon --in ' inputPLYname ' --out ' outputPLYname ' --width ' num2str(w) ' --density' ' --verbose']);

if exist('commandWindow.txt','file')
    delete('commandWindow.txt');
end
clc;
diary('commandWindow.txt');
diary ON;
system(['SurfaceTrimmer --in ' outputPLYname ' --out ' outputPLYname ' --trim ' num2str(0) ' --smooth 0' ' --verbose']);
diary OFF;
fid = fopen('commandWindow.txt');
while ~feof(fid)
    tline = fgetl(fid);
end
tline = split(tline,'Value Range: [');
tline = split(tline{2},'] ');
tline = split(tline{1},',');
Dmin = str2double(tline{1});
Dmax = str2double(tline{2});
m = (0.35-0.30)/(4.999921-7.029424);
c = ((4.999921*0.30)-(7.029424*0.35))/(4.999921-7.029424);
den = (m*Dmax)+c;
d = Dmax - log2(1/den);
fclose(fid);

if exist('commandWindow.txt','file')
    delete('commandWindow.txt');
end

system(['SurfaceTrimmer --in ' outputPLYname ' --out ' outputPLYname ' --trim ' num2str(d) ' --smooth 0' ' --verbose']);

copyfile(outputPLYname,[outputDir '/' outputPLYname],'f');
delete(inputPLYname);
delete(outputPLYname);

cd(currentFolder);
waitbar(0.9,hw);
[vertices,faces] = F_readPLY([outputDir '/' outputPLYname]);

waitbar(1,hw);
close(hw);
clear hw;

%------------- END CODE --------------
end
