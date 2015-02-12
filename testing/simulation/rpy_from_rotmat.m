function [rpy] = rpy_from_rotmat(R)
% RPY_FROM_ROTMAT Roll, pitch, yaw Euler angles from rotation matrix.
%
%   [rpy] = RPY_FROM_ROTMAT(R) computes roll, pitch and yaw angles from the
%   rotation matrix 'R'.  The pitch angle 'p' is constrained to the range
%   (-pi/2, pi/2].  The returned angles are in radians.
%
%   Inputs:
%   -------
%    R  - 3x3 orthonormal rotation matrix.
%
%   Outputs:
%   --------
%    rpy  - 3x1 vector of roll, pitch, yaw Euler angles.

rpy = zeros(3, 1);

% Roll.
rpy(1) = atan2(R(3, 2), R(3, 3));

% Pitch.
sinp = -R(3, 1);
cosp = sqrt(R(1, 1).*R(1, 1) + R(2, 1).*R(2, 1));

if abs(cosp) > 1e-13  % Or maybe this should be exactly zero?
  rpy(2) = atan2(sinp, cosp);
else
  rpy(2) = pi/2;
  
  if sinp < 0
    rpy(2) = -rpy(2);
  end
end

% Yaw.
rpy(3) = atan2(R(2, 1), R(1, 1));