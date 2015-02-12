function [ J ] = dgdy(point, calibParams)
%DGDY Evaluates the jacobian of the inverse stereo camera model (See
%Barfoot AER1514 Lecture 15 notes

c_u = calibParams.c_u;
c_v = calibParams.c_v;
f_u = calibParams.f_u;
f_v = calibParams.f_v;
b = calibParams.b;

M = [f_u 0 c_u 0;
    0 f_v c_v 0;
    f_u 0 c_u -f_u*b;
    0 f_v c_v 0];

y = (1/point(3))*M*cart2homo(point);
u_l = y(1);
v_l = y(2);
u_r = y(3);
v_r = y(4);

row1 = [-u_r + c_u, 0, u_l - c_u, 0];
row2 = [-f_u/f_v*(0.5*(v_l + v_r) - c_v), 0.5*f_u/f_v*(u_l - u_r),f_u/f_v*(0.5*(v_l + v_r) - c_v), 0.5*f_u/f_v*(u_l - u_r)];
row3 = [-f_u, 0, f_u, 0];

J = b/(u_l - u_r)^2*[row1; row2; row3];
    

end

