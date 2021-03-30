function F_closeSaveAndDownsample(rawWriteID,fDetWriteID)
%F_closeSaveAndDownsample(rawWriteID,fDetWriteID) closes the output files
%
%   Inputs:
%       rawWriteID - handle of raw data file
%       fDetWriteID - handle of details file
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

fclose(rawWriteID);
fclose(fDetWriteID);

%------------- END CODE --------------

end

