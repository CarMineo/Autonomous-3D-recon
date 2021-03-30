function F_saveIndDsData(outputDir,iDsData)
%F_saveIndDsData(outputDir,iDsData)
%saves the indices of the down-sampled point cloud.
%
%   Inputs: 
%       iDsData - indices of points belonging to the initial downsamples cloud
%       outputDir - string containing the path of the output directory
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

indWriteID = fopen([outputDir '/iDsData.dat'],'w');
fwrite(indWriteID,iDsData,'uint64');
fclose(indWriteID);

%------------- END CODE --------------

end

