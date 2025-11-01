function [isRotationMatrix] = IsRotationMatrix(R)
% The function checks that the input R is a valid rotation matrix, that is 
% a valid element of SO(3).
% Return true if R is a valid rotation matrix, false otherwise. In the
% latter case, print a warning pointing out the failed check.

    isRotationMatrix = false;
    count = 0;
    
    for i=1:3
        if(R(i,i) - 1 >= 1e-3)
            warning("R is not a rotation matrix");
            break;
        else 
            count = count + 1;
        end
    end
     
    if(abs(det(R) - 1) < 1e-3 && count == 3)
        isRotationMatrix = true;
    end

end