function [T_21] = scalarWeightedPointCloudAlignment(p_f1_1, p_f2_2)

weights = zeros(1, size(p_f1_1,2));
for i=1:size(p_f1_1,2)
    weights(i) = 1;% 1/(p_f1_1(:,i)'*p_f1_1(:,i));
end

w = sum(weights);

p1 = zeros(3,1);
p2 = zeros(3,1);
A = zeros(3,3);

for i=1:size(p_f1_1,2)
    p1 = p1 + weights(i)*p_f1_1(:,i)/w;
    p2 = p2 + weights(i)*p_f2_2(:,i)/w;
end
for i=1:size(p_f1_1,2)
    A = A + weights(i)*(p_f2_2(:, i) - p2)*(p_f1_1(:,i) - p1)';
end
W = A/w;

[V,~,U] = svd(W);

C_21 = V*[1 0 0; 0 1 0; 0 0 det(U)*det(V)]*U';
r_21_1 = -C_21'*p2 + p1;

%Final 
T_21 = [C_21 -C_21*r_21_1; 0 0 0 1];
end

