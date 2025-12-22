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
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control

            bTt = self.gm.getToolTransformWrtBase();
            t_r_g = bTg(1:3,4) - bTt(1:3,4);
            v_dot = self.k_l * t_r_g;

            gRt = bTg(1:3,1:3) * bTt(1:3,1:3)';
            t_rho_g = rotm2axang(gRt);
            omega_dot = self.k_a * t_rho_g(1:3) * t_rho_g(4);
            
            x_dot = [v_dot; omega_dot'];
            
        end
    end
end

