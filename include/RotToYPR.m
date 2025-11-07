function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Check that R is a valid rotation matrix using IsRotationMatrix().

    if ~IsRotationMatrix(R)
        error('Input matrix R is not a valid rotation matrix');
    end

    theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));

    if abs(cos(theta)) > 1e-6
        psi = atan2(R(2,1), R(1,1));
        phi = atan2(R(3,2), R(3,3));
    else
        psi = atan2(-R(1,2), R(2,2));
        phi = 0;
    end

    % Normalize angles to [0, 2*pi)
    if psi < 0
        psi = psi + 2*pi;
    end
    if phi < 0
        phi = phi + 2*pi;
    end
end
