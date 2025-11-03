%% TO DO: Test assignment 1 MCM 2024-2025
% 1.1 Angle-axis to rot

    addpath("include");

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
    
    addpath("include");

    mat1 = [1,0,0;0,0,-1;0,1,0];
    mat2 = [0.5,-sqrt(3)/2,0;sqrt(3)/2,0.5,0;0,0,1];
    mat3 = [1,0,0;0,1,0;0,0,1];
    mat4 = [-1,0,0;0,-1,0;0,0,1];
    mat5 = [-1,0,0;0,1,0;0,0,1];

    [h1_r, theta1_r] = RotToAngleAxis(mat1);
    [h2_r, theta2_r] = RotToAngleAxis(mat2);
    [h3_r, theta3_r] = RotToAngleAxis(mat3);
    [h4_r, theta4_r] = RotToAngleAxis(mat4);
    [h5_r, theta5_r] = RotToAngleAxis(mat5);
        
    

%% 1.3 Euler to rot

    addpath("include");

    psi1 = 0;
    theta1 = 0;
    phi1 = pi/2;

    psi2 = pi/3;
    theta2 = 0;
    phi2 = 0;

    psi3 = pi/3;
    theta3 = pi/2;
    phi3 = pi/4;

    psi4 = 0;
    theta4 = pi/2;
    phi4 = pi/12;

    % Convert Euler angles to rotation matrices
    R1_euler = YPRToRot(psi1, theta1, phi1);
    R2_euler = YPRToRot(psi2, theta2, phi2);
    R3_euler = YPRToRot(psi3, theta3, phi3);
    R4_euler = YPRToRot(psi4, theta4, phi4);

%% 1.4 Rot to Euler

    addpath("include");

    R1_toEuler = [1, 0, 0; 0, 0, -1; 0, 1, 0];
    R2_toEuler = [1/2, -sqrt(3)/2, 0; sqrt(3)/2, 1/2, 0; 0, 0, 1];
    R3_toEuler = [0, -sqrt(2)/2, sqrt(2)/2; 0.5, (sqrt(2)*sqrt(3))/4, (sqrt(2)*sqrt(3))/4;
        -sqrt(3)/2, sqrt(2)/4, sqrt(2)/4];

    [psi1_fromR, theta1_fromR, phi1_fromR] = RotToYPR(R1_toEuler);
    [psi2_fromR, theta2_fromR, phi2_fromR] = RotToYPR(R2_toEuler);
    [psi3_fromR, theta3_fromR, phi3_fromR] = RotToYPR(R3_toEuler);

%% 1.5 Rot to angle-axis with eigenvectors

    addpath("include");

    R1_eigen = [1,0,0;0,0,-1;0,1,0];
    R2_eigen = 1/9*[4,-4,-7;8,1,4;-1,-8,4];

    % Method 1

    [h1_m1, theta1] = RotToAngleAxis(R1_eigen);
    [h2_m1, theta2] = RotToAngleAxis(R2_eigen);

    % Method 2

    % D1 e D2 autovalori in diagionale, V1 e V2 hanno colonne con gli
    % autovettori corrispondenti alle posizioni di D

    [V1, D1] = eig(R1_eigen);
    [V2, D2] = eig(R2_eigen);

    function eigenV = findUnitEigenvector(V,D)

        for i=1:3
            if(D(i,i) - 1 < 1e-3)
                eigenV = V(:,i);
                break;
            else
                eigenV = NaN;
            end
        end
    
    end

    h1_m2 = findUnitEigenvector(V1,D1);
    h2_m2 = findUnitEigenvector(V2,D2);





    