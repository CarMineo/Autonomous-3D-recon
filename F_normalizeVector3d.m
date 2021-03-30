function vn = F_normalizeVector3d(v)
%vn = F_normalizeVector3d(v)
%Normalizes a 3D vector to have norm equal to 1
%Returns the normalization of vector V, such that ||V|| = 1. Vector V is
%given as a row vector.
%
%   If V is a N-by-3 array, normalization is performed for each row of the
%   input array.
%
%   See also:
%   vectors3d, vectorNorm3d
%
%   ---------
%   author : David Legland 
%   INRA - TPV URPOI - BIA IMASTE
%   created the 29/11/2004.
%

% HISTORY
% 2005-11-30 correct a bug
% 2009-06-19 rename as normalizeVector3d
% 2010-11-16 use bsxfun (Thanks to Sven Holcombe)

vn   = bsxfun(@rdivide, v, sqrt(sum(v.^2, 2)));
