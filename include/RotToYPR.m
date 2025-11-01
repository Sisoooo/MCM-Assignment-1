function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Check that R is a valid rotation matrix using IsRotationMatrix().

    if ~IsRotationMatrix(R)
        error('Input matrix R is not a valid rotation matrix');
    end

    theta = atan2(-R(3,1), sqrt(R(1,1)^2) + R(2,1)^2);
    if cos(theta) ~= 0
        psi = atan2(R(2,1), R(1,1));
        phi = atan2(R(3,2), R(3,3));
    else
        error("Singular configuration");
    end

end

