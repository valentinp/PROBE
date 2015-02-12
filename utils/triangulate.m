function [p_fc_c] = triangulate(yMeas, calibParams)
% triangulate Triangulates 3D point from stereo camera measurement

u_l = yMeas(1);
v_l = yMeas(2);
u_r = yMeas(3);
v_r = yMeas(4);

c_u = calibParams.c_u;
c_v = calibParams.c_v;
f_u = calibParams.f_u;
f_v = calibParams.f_v;
b = calibParams.b;


z = b*f_u/(u_l - u_r);
x = (u_l-c_u)*z/f_u;
y = (0.5*(v_l+v_r)-c_v)*z/f_v;

p_fc_c = [x; y; z];

% p_fc_c = b/(u_l - u_r)*[0.5*(u_l) - c_u; ...
%                         f_u/f_v*(0.5*(v_l + v_r) - c_v); ...
%                         f_u];

%p_fc_c(1) = p_fc_c(1) - b/2;        
end
