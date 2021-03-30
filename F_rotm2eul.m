function EUL = F_rotm2eul(R,unit)
%EUL = F_rotm2eul(R,unit) computes the Euler angles from a rotation matrix
%
%   Inputs:
%       R = [R11 R12 R13]
%           [R21 R22 R23]
%           [R31 R32 R33]
%       unit - unit required for output angles ('deg' for degrees or 'rad for radians')
%   
%   Outputs:
%       EUL - [3x1] set of Eulerian angles
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 30-March-2021
% Tested with: Matlab 2020b


%------------- BEGIN CODE --------------

R11 = R(1,1,:); R11 = R11(:);
R12 = R(1,2,:); R12 = R12(:);
R13 = R(1,3,:); R13 = R13(:);
R21 = R(2,1,:); R21 = R21(:);
R22 = R(2,2,:); R22 = R22(:);
R23 = R(2,3,:); R23 = R23(:);
R31 = R(3,1,:); R31 = R31(:);

sa = R21 ./ sqrt(R11.^2 + R21.^2);
ca = R11 ./ sqrt(R11.^2 + R21.^2);

B_out = atan2(-R31, ca.*R11 + sa.*R21);

idx90 = abs(abs(B_out) - pi/2) < 1e-4 * pi/2; % i.e. abs(B_out) == pi/2
A_out(idx90,:) = 0;
A_out(~idx90,:) = atan2(R21(~idx90), R11(~idx90));

C_out(idx90,:) = sign(B_out(idx90)) .* atan2(R12(idx90), R22(idx90));
C_out(~idx90,:) = atan2( sa(~idx90).*R13(~idx90) - ca(~idx90).*R23(~idx90), ...
                        -sa(~idx90).*R12(~idx90) + ca(~idx90).*R22(~idx90));

if strcmp(unit,'deg')
    A_out = rad2deg(A_out);
    B_out = rad2deg(B_out);
    C_out = rad2deg(C_out);
end

EUL = [A_out B_out C_out];

%------------- END CODE --------------

end

