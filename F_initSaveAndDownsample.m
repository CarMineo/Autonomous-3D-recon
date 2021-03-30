function [rawWriteID,fDetWriteID,iDsData] = F_initSaveAndDownsample(outputDir)
%[rawWriteID,fDetWriteID,iDsData] = F_initSaveAndDownsample(outputDir)
%initializes memory-mapping for data saving, down-sampling and merging
%
%   Inputs:
%       outputDir - string containing the path of the output directory
%
%   Outputs:
%       rawWriteID - handle of file for storing the raw data
%       fDetWriteID - handle of file for storing the details
%       iDsData - indices of points belonging to the initial downsamples cloud
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

fDetWriteID = fopen([outputDir '/fDetData.dat'], 'w');
rawWriteID = fopen([outputDir '/rawData.dat'], 'w');
iDsData = uint64(zeros(0,0));

%------------- END CODE --------------

end

