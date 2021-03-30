function R = F_eul2rotm(EUL,unit)
%R = F_eul2rotm(EUL,unit) computes the rotation matrix corresponding to a
%set of Eulerian angles.
%
%   Inputs:
%       EUL - [3x1] set of Eulerian angles
%       unit - unit of input angles ('deg' for degrees or 'rad for radians')
%   
%   Outputs:
%       R = [R11 R12 R13]
%           [R21 R22 R23]
%           [R31 R32 R33]
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

R = zeros(3,3,size(EUL,1));

A = EUL(:,1);
B = EUL(:,2);
C = EUL(:,3);

if strcmp(unit,'deg')
    sinA = sind(A); cosA = cosd(A);
    sinB = sind(B); cosB = cosd(B);
    sinC = sind(C); cosC = cosd(C);
elseif strcmp(unit,'rad')
    sinA = sin(A); cosA = cos(A);
    sinB = sin(B); cosB = cos(B);
    sinC = sin(C); cosC = cos(C);
end

R11 = cosA.*cosB;
R12 = (cosA.*sinB.*sinC) - (sinA.*cosC);
R13 = (cosA.*sinB.*cosC) + (sinA.*sinC);
R21 = sinA.*cosB;
R22 = (sinA.*sinB.*sinC) + (cosA.*cosC);
R23 = (sinA.*sinB.*cosC) - (cosA.*sinC);
R31 = -sinB;
R32 = cosB.*sinC;
R33 = cosB.*cosC;

R(1,1,:) = R11;
R(1,2,:) = R12;
R(1,3,:) = R13;
R(2,1,:) = R21;
R(2,2,:) = R22;
R(2,3,:) = R23;
R(3,1,:) = R31;
R(3,2,:) = R32;
R(3,3,:) = R33;

%------------- END CODE --------------

end

