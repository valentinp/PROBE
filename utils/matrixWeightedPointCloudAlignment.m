function [ T_21] = matrixWeightedPointCloudAlignment(p_f1_1, p_f2_2, R_1, R_2, T_21_est, calibParams, optParams)
%MATRIXWEIGHTEDPOINTCLOUDALIGNMENT Performs a matrix weighted point cloud
%alignment and returns a 4x4 transformation matrix

%MOTION_EST_MATRIX 
%Estimate the motion based on Matrix weights
%See AER1514 Lecture 15 for details

%p_f1_1 and p_f2_2 are 3xN matrices where every ith column represents the 3D location
%of a landmark in the previous and current frames

%R_1 and R_2 are 3x3xN 3D matrices that represent the co-variance 
%matrices of the ith landmark in the current and previous frames



%No of landmarks
numLandmarks = size(p_f1_1, 2);

C_21 = T_21_est(1:3,1:3);
T_12_est = inv(T_21_est);
r_21_1 = T_12_est(1:3,4);



i = 1;
maxIterations = optParams.maxGNIter;
converged = false;

G2 = NaN(3,4, numLandmarks);
G1 = NaN(3,4, numLandmarks);
for j=1:numLandmarks
    G2(:,:,j) = dgdy(p_f2_2(:,j), calibParams);
    G1(:,:,j) = dgdy(p_f1_1(:,j), calibParams);
end        

Jold = 0;
while ~converged && i < maxIterations

    LS = zeros(6,6);
    RS = zeros(6,1);    
    for j=1:numLandmarks
        
        e_j = p_f2_2(1:3,j) - C_21*(p_f1_1(1:3,j) - r_21_1);
        E_j = [C_21 -crossMat(C_21*(p_f1_1(1:3,j) - r_21_1))]; 
       
        G2_j = G2(:,:, j);
        G1_j = G1(:,:, j);
        
        Sigma_j = (G2_j*R_2(:,:,j)*G2_j' + C_21*G1_j*R_1(:,:,j)*G1_j'*C_21');
        
        %Build up Ej'*Wj*Ej and -Ej'*W'ej; the left and right sides
        %Use LM if lambda > 0
        LS = LS + E_j'*(Sigma_j\E_j) + optParams.LMlambda*diag(diag(E_j'*(Sigma_j\E_j)));
        RS = RS + E_j'*(Sigma_j\e_j);
        %Keep track of cost function in the first iteration
        if i == 1
            Jold = Jold + 0.5*e_j'*(Sigma_j\e_j);
        end
    end
    


   xi =LS\(-RS); 
   
    xi_star = optParams.lineLambda*xi;
    phi = xi_star(4:6);
    eps = xi_star(1:3);

    C_21 = expm(-crossMat(phi))*C_21;
    r_21_1 = r_21_1 + eps;
    
            %Calculate cost
        J = 0;
        for j=1:numLandmarks
            e_j = p_f2_2(1:3,j) - C_21*(p_f1_1(1:3,j) - r_21_1);
            G2_j = G2(:,:, j);
            G1_j = G1(:,:, j);
            Sigma_j = (G2_j*R_2(:,:,j)*G2_j' + C_21*G1_j*R_1(:,:,j)*G1_j'*C_21');
            J = J + 0.5*e_j'*(Sigma_j\e_j);
        end

%   %Perform line search
%    while lineLambda > 0.3
%         
%         %Solve for estimate
%         xi_star = lineLambda*xi;
% 
%         phi = xi_star(4:6);
%         eps = xi_star(1:3);
% 
%         C_21_temp = C_21*expm(-crossMat(phi));
%         r_21_1_temp = r_21_1 + eps;
% 
% 
%         %Calculate cost
%         J = 0;
%         for j=1:numLandmarks
%             e_j = p_f2_2(1:3,j) - C_21*(p_f1_1(1:3,j) - r_21_1);
%             G2_j = G2(:,:, j);
%             G1_j = G1(:,:, j);
%             Sigma_j = (G2_j*R_2(:,:,j)*G2_j' + C_21*G1_j*R_1(:,:,j)*G1_j'*C_21');
%             J = J + 0.5*e_j'*(Sigma_j\e_j);
%         end
%         
%         %Ensure cost goes down
%         if J > Jold
%             lineLambda = lineLambda - 0.1;
%         else
%             Jold = J;
%             C_21 = C_21_temp;
%             r_21_1 = r_21_1_temp;
%             break;
%         end
%    end
    if abs(Jold - J)/(Jold) < 0.01
        converged = true;
    end
    Jold = J;
    
    i=i+1;
end
T_21 = [C_21 -C_21*r_21_1; 0 0 0 1];

end

