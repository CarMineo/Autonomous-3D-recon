function intensityFact = F_intensityFactor(sensor,points)
%intensityFact = F_intensityFactor(sensor,points)
%computes the intensity factor for points (given with respect to the sensor
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
%       intensityFact - [nx1 double] intensity factors
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

resAz = sensor.resAz;
resEl = sensor.resEl;

aXmax = sensor.azRange(2);
aYmax = sensor.elRange(2);

xv = tan(aXmax);
yv = tan(aYmax);
zv = sqrt(1 - (xv.^2) - (yv.^2));
[THv,~,~] = cart2sph(xv,yv,zv);

[TH,~,~] = cart2sph(points(:,1),points(:,2),points(:,3));
depth = points(:,3);
aX = atan2(points(:,1),depth);
aY = atan2(points(:,2),depth);
d = zeros(size(aX));

d((abs(aX)<=aXmax) & (abs(aY)<=aYmax)) = points(((abs(aX)<=aXmax) & (abs(aY)<=aYmax)),3);
d(((aX>aXmax) & (aX<(pi-aXmax))) & ((TH>=-THv) & (TH<THv))) = points((((aX>aXmax) & (aX<(pi-aXmax))) & ((TH>-THv) & (TH<THv))),1)./tan(aXmax);
d(((aY>aYmax) & (aY<(pi-aYmax))) & ((TH>=THv) & (TH<-THv+pi))) = points((((aY>aYmax) & (aY<(pi-aYmax))) & ((TH>=THv) & (TH<-THv+pi))),2)./tan(aYmax);
d(((aX<-aXmax) & (aX>(-pi+aXmax))) & (((TH>=-THv+pi) & (TH<=pi)) | ((TH<-pi+THv) & (TH>=-pi)))) = -points((((aX<-aXmax) & (aX>(-pi+aXmax))) & (((TH>=-THv+pi) & (TH<=pi)) | ((TH<-pi+THv) & (TH>=-pi)))),1)./tan(aXmax);
d(((aY<-aYmax) & (aY>(-pi+aYmax))) & ((TH<-THv) & (TH>=THv-pi))) = -points((((aY<-aYmax) & (aY>(-pi+aYmax))) & ((TH<-THv) & (TH>=THv-pi))),2)./tan(aYmax);
d(((aX<=(-pi+aXmax)) | (aX>=(pi-aXmax))) & ((aY<=(-pi+aYmax)) | (aY>=(pi-aYmax)))) = -points((((aX<=(-pi+aXmax)) | (aX>=(pi-aXmax))) & ((aY<=(-pi+aYmax)) | (aY>=(pi-aYmax)))),3);

surface = 4.*(d.^2).*tan(aXmax).*tan(aYmax);
intensityFact = (resAz*resEl)./surface;
if sensor.optimumDist < sensor.optimumDist_sampling
    intensityFact = intensityFact.*(sensor.optimumDist/sensor.optimumDist_sampling);
end

%------------- END CODE --------------

end

