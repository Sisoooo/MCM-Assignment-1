function [eRt] = rotation(psi, theta, phi)
%ROTATION Summary of this function goes here
%   Detailed explanation goes here
    
    R_z = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0,0,1];
    R_y = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    R_x = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    
    eRt = R_z * R_y * R_x;
    
end