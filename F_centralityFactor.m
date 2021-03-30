function centrFact = F_centralityFactor(sensor,points)
%centrFact = F_centralityFactor(sensor,rotLines) computes the centrality
%factor (sigma) for the points (given with respect to the sensor
%reference system)
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
%       points - [nx3 double] array or points coordinates 
%
%   Outputs:
%       centrFact - [nx1 double] centrality factors
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

aXmax = sensor.azRange(2);
aYmax = sensor.elRange(2);

depth = points(:,3);
aX = atan2(points(:,1),depth);
aY = atan2(points(:,2),depth);

centrFactX = -((1/aXmax).*abs(aX)) + 1;
centrFactY = -((1/aYmax).*abs(aY)) + 1;

centrFact = zeros(size(centrFactX));
centrFact(abs(aX)<=pi/2 & abs(aY)<=pi/2) = min([centrFactX(abs(aX)<=pi/2 & abs(aY)<=pi/2) centrFactY(abs(aX)<=pi/2 & abs(aY)<=pi/2)],[],2);
centrFact(abs(aX)>pi/2 & abs(aY)>pi/2) = max([centrFactX(abs(aX)>pi/2 & abs(aY)>pi/2) centrFactY(abs(aX)>pi/2 & abs(aY)>pi/2)],[],2);

centrFactXmin = -((1/aXmax).*abs(pi)) + 1;
centrFactYmin = -((1/aYmax).*abs(pi)) + 1;
if centrFactYmin <= centrFactXmin
    corrFactor = (centrFactYmin+1)/(centrFactXmin+1);
else
    corrFactor = (centrFactXmin+1)/(centrFactYmin+1);
end
centrFact(abs(aX)>pi/2 & abs(aY)>pi/2) = centrFact(abs(aX)>pi/2 & abs(aY)>pi/2).* corrFactor;

%------------- END CODE --------------

end

