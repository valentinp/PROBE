function [T_21] = getTransformation(state1, state2)
%GETTRANSFORMATION Returns the 4x4 transformation matrix

R_21 = state2.C_vi*(state1.C_vi)';
p_21_1 = state1.C_vi*(state2.r_vi_i - state1.r_vi_i);

T_21 = [R_21 -R_21*p_21_1; 0 0 0 1];

end

