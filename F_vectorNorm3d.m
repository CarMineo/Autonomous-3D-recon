function n = F_vectorNorm3d(v)
%n = F_vectorNorm3d(v)
%Norm of a 3D vector or of set of 3D vectors. Returns the norm of vector V.
%When V is a N-by-3 array, compute norm for each vector of the array.
%Vector are given as rows. Result is then a N-by-1 array.
%
%   NOTE: computes only euclidean norm.
%
%   See Also
%   vectors3d, normalizeVector3d, vectorAngle3d, hypot3
%
%   ---------
%   author : David Legland 
%   INRA - TPV URPOI - BIA IMASTE
%   created the 21/02/2005.

%   HISTORY
%   19/06/2009 rename as vectorNorm3d

n = sqrt(sum(v.*v, 2));
