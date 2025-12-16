%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs: i, joint index;
            % The function returns bJi

            % Compute angular Jacobian
            for j=1:i
                if self.gm.jointType(j) == 0
                    k_index = self.gm.getTransformWrtBase(j);
                    bJ_a(:, j) = k_index(1:3,3);
                elseif self.gm.jointType(j) == 1
                    bJ_a(:, j) = zeros(3,1);
                end
            end

            % Compute linear Jacobian
            for j=1:i
                if self.gm.jointType(j) == 0
                    nR0 = self.gm.getTransformWrtBase(length(self.gm.jointNumber));
                    k_index = self.gm.getTransformWrtBase(j);
                    bJ_l(:, j) = cross(k_index(1:3,3), nR0(1:3,4)-k_index(1:3,4));
                elseif self.gm.jointType(j) == 1
                    k_index = self.gm.getTransformWrtBase(j);
                    bJ_l(:, j) = k_index(1:3,3);
                end
            end

            bJi = [bJ_a; bJ_l]; 
            
        end


        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix
        % TO DO
        
        self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
        end
    end
end

