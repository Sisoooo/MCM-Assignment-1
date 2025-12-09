%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0,jointType);

%% Q1.3
bTe = geometricModel.getTransformWrtBase(length(jointType));
disp('bTe')
disp(bTe)

T_0 = geometricModel.iTj;
t62 = eye(4);

% Must reverse: 6 -> 2, not 2 -> 6
for i=3:6
    t62 = t62 * T_0(:,:,i);
end

t26 = [t62(1:3,1:3)', -t62(1:3,1:3)' * t62(1:3,4); 0 0 0 1];

disp('t26')
disp(t26)

%% Q1.4 Simulation
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi);
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)

%% Q1.5

qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];
geometricModel.updateDirectGeometry(qf);
km = kinematicModel(geometricModel);
jac = km.getJacobianOfLinkWrtBase(6);

disp('Basic robot Jacobian for link 6:')
disp(jac)


%% Q1.6

qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];
geometricModel.updateDirectGeometry(qf);
km = kinematicModel(geometricModel);
updateJacobian(km)

disp('End-Effector basic robot Jacobian:')
disp(km.J)

%% Q1.7

qp = [0.7, -0.1, 1, -1, 0, 0.03, 1.3];
qv = [0.9, 0.1, -0.2, 0.3, -0.8, 0.5, 0];

geometricModel.updateDirectGeometry(qp);
km = kinematicModel(geometricModel);
updateJacobian(km);

ni = km.J * qv';
omega_ee = ni(1:3);
v_ee = ni(4:6);

disp('Angular velocity components:')
disp(omega_ee);

disp('Linear velocity components:')
disp(v_ee);