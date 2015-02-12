function [R] = rotmat_from_rpy(rpy)
% ROTMAT_FROM_RPY Rotation matrix from roll, pitch, yaw Euler angles.
%
%   [R] = ROTMAT_FROM_RPY(rpy) produces a 3x3 orthonormal rotation matrix R
%   from the vector rpy containing roll angle r, pitch angle p, and yaw angle
%   y.  All angles are specified in radians.  We use the aerospace convention
%   here (see descriptions below).  Note that roll, pitch and yaw angles are
%   also often denoted by phi, theta and psi (respectively).
%
%   The angles are applied in order according to the following:
%
%     1.  Yaw   -> by angle 'y' in the local (body-attached) frame.
%     2.  Pitch -> by angle 'p' in the local frame.
%     3.  Roll  -> by angle 'r' in the local frame.  
%
%   Note that this is exactly equivalent to the following fixed-axis
%   rotations:
%
%     1.  Roll  -> by angle 'r' in the fixed frame.
%     2.  Pitch -> by angle 'p' in the fixed frame.
%     3.  Yaw   -> by angle 'y' in the fixed frame.
%
%   Inputs:
%   -------
%    rpy  - 3x1 vector of roll, pitch, yaw Euler angles.
%
%   Outputs:
%   --------
%    R  - 3x3 orthonormal rotation matrix.

r = rpy(1);
p = rpy(2);
y = rpy(3);

R = [cos(y)*cos(p), ...
     cos(y)*sin(p)*sin(r) - sin(y)*cos(r), ...
     cos(y)*sin(p)*cos(r) + sin(y)*sin(r);

     sin(y)*cos(p), ...
     sin(y)*sin(p)*sin(r) + cos(y)*cos(r), ...
     sin(y)*sin(p)*cos(r) - cos(y)*sin(r);

    -sin(p), cos(p)*sin(r), cos(p)*cos(r)];