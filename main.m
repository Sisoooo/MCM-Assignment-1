addpath('include');

%% TO DO: Test assignment 1 MCM 2024-2025
% 1.1 Angle-axis to rot

    h1 = [1;0;0];
    theta1 = pi/2;
  
    h2 = [0;0;1];
    theta2 = pi/3;
    
    % Third test vector given as rotation vector [-pi/3,-pi/6, pi/3]
    % Given that the rotation vector is the product between theta and h
    % and knowing that h has unitary norm, we calculate the rotation
    % vector's norm and divide the h vector with it
    
    h3 = [-pi/3,-pi/6, pi/3];
    norm_h3 = 0;
    
    for i=1:length(h3)
        norm_h3 = norm_h3 + h3(i)^2;
    end
    
    theta3 = sqrt(norm_h3);
    h3 = h3/theta3;
    
    % Testing method 1.1
    
    R1 = AngleAxisToRot(h1, theta1); 
    R2 = AngleAxisToRot(h2, theta2);
    R3 = AngleAxisToRot(h3, theta3);

%% 1.2 Rot to angle-axis

    mat1 = [1,0,0;0,0,-1;0,1,0];
    mat2 = [0.5,-sqrt(3)/2,0;sqrt(3)/2,0.5,0;0,0,1];
    mat3 = [1,0,0;0,1,0;0,0,1];
    mat4 = [-1,0,0;0,-1,0;0,0,1];
    mat5 = [-1,0,0;0,1,0;0,0,1];
    
    

%% 1.3 Euler to rot

%% 1.4 Rot to Euler

%% 1.5 Rot to angle-axis with eigenvectors