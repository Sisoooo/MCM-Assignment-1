function [h,theta] = RotToAngleAxis(R)
% Given a rotation matrix this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% Check that R is a valid rotation matrix using IsRotationMatrix()

    if ~IsRotationMatrix(R)
        error('Input matrix R is not a valid rotation matrix');
    end

    theta= acos((trace(R)-1)/2);
    if theta == 0
        h = [1; 0; 0]; % arbitrary vector when theta is 0
    elseif theta == pi
        % define h for the case when theta is pi
        h=zeros(3:1);
        for i=1:3
            h(i)=sqrt((R(i,i)+1)/2);
        end
        
        % For cycle to choose an index to compute our solutions from
        % We choose the first possible index for simplicity
        ch_index = 0;
        for i=1:3
            if abs(sqrt((R(i,i)+1)/2)) > 0
                ch_index = i;
                break;
            end
        end
        
        % We arbitrarily choose the positive solution for the h vector
        % in ch_index, so no modifications are required

        for j=1:3
            if j ~= ch_index
               h(j) = sgn(h(ch_index))*sgn(R(ch_index,j))*sqrt((R(i,i)+1)/2);
            end
        end
    else
        % define h for any other theta
        h = (1/sin(theta))*vex((R-R')/2);
    end
end

% Sign function to be used in the case theta=pi of the previous function
function s = sgn(x)
    
    if x > 0
            s = 1;
        elseif x < 0
            s = -1;
        else
            s = 0;
    end

end

 
function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)

a=[S_a(3,2); S_a(1,3); S_a(2,1)];

end