%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [b_e, x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control

            bTt = self.gm.getToolTransformWrtBase();
            t_r_g = bTg(1:3,4) - bTt(1:3,4);
        
            tRg = bTt(1:3,1:3)' * bTg(1:3,1:3);
            [h, theta] = RotAngleAxis(tRg);
            t_rho_g = (h * theta); 
            
            % projected in b frame
            rho = bTt(1:3,1:3) * t_rho_g;

            b_e = [rho; t_r_g];
        
            lambda = [self.k_a * eye(3), zeros(3,3); zeros(3,3), self.k_l * eye(3)];
            x_dot = lambda * b_e;
            
        end
    end
end

