function R = AngleAxisToRot(h,theta)

    % The fuction implement the Rodrigues Formula
    % Input: 
    % h is the axis of rotation
    % theta is the angle of rotation (rad)
    % Output:
    % R rotation matrix
    
    % 3x3 identity matrix 
    I = eye(3);
    
    % Matrix h-cross derived by vector h
    K = [0 -h(3) h(2); h(3) 0 -h(1); -h(2) h(1) 0];

    % Rodrigues' formula to obtain 
    R = I + sin(theta) * K + (1 - cos(theta)) * (K^2);

end
