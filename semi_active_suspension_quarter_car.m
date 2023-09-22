function [Zdot, O_simulator, O_model] = semi_active_suspension_quarter_car(t, z, input)
%active_suspension_quarter_car Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the control action and interpolates
%   road inputs and it passes these as scalar values to the vehicle model.

%% Initialization : Augmented state variables

% Model States
q = [z(1), z(2), z(3), z(4)]';

% Controller (Active Damper) States
Z_cont_qcar = [z(5:end)']';

% Displacements
z_s = z(1);
z_u = z(2);

% Velocities
z_dot_s = z(3);
z_dot_u = z(4);

% Damper piston velocity (Used for damping force saturation)
damper_piston_velocity = z_dot_u - z_dot_s;

%% Initialization : Road Input

z_r = interp1(input.time, input.z_r, t, 'pchip');

%% Initialization : Controll Matrices

% Sprung Mass Position Controller
% -- A -- Controller Cannonical State Matrix 
Ac_ds = input.cA_ds;

% -- B -- Controller Cannonical Input Matrix
Bc_ds = input.cB_ds;

% -- C -- Controller Cannonical Output Matrix
Cc_ds = input.cC_ds;

% -- D -- Controller Cannonical Outpu<> Input Matrix
Dc_ds = input.cD_ds;

%% Reference Signal to be tracked

% Here we will create the reference signal that the controller must track
% In our case, this implies the position of the sprung mass that must be
% tracked
% Since our goal is to keep the sprung mass as level as possible, the
% displacement of the sprung mass must be 0
% The displacement being 0 implies that the position of the sprung mass
% must not change which implies the position is equal to the steady-state
% value calculated
z_s_ref = 0;

%% Error 

e_zs = z_s_ref - (z_s - input.zs_steady_state);

%% Controller Action - Desired Controller Force

% Controller output
y_cont = Cc_ds*Z_cont_qcar + Dc_ds*e_zs;

% Desired controller force
F_active_damper = input.controller_switch * y_cont;

%% Inverse Controller Model - Realizeable Controller Force

% Minimum damping cofficient (Reference - Tenecco Active Damper)
damping_min = 100; % Ns/m

% Maximum damping cofficient (Reference - Tenecco Active Damper)
damping_max = 10000; % Ns/m

% Required damping coefficient (Calculated using required damping force)
damping_required = F_active_damper / damper_piston_velocity;

% At steady-state damping pistion will be at rest and hence damping is
% infinity. Hence, we need to correct for that
if ~isinf(damping_required) && ~isnan(damping_required)

    if abs(damping_required) > damping_max
        damping_required = sign(damping_required)*damping_max;
%     elseif abs(damping_required) < damping_min  
%         damping_required = sign(damping_required)*damping_min;
    end

else
    damping_required = 0;
end

% Realizable Controller Force
F_active_damper = damping_required * damper_piston_velocity;

%% Quarter car system dynamics

[Qdot, ~, ~, O_model] = quarter_car_model_linear(q, input, F_active_damper, z_r);

%% Controller Dynamics

Z_dot_cont_qcar = Ac_ds*Z_cont_qcar + Bc_ds*e_zs;

%% Augmented system dynamics

Zdot = [Qdot;
        Z_dot_cont_qcar
        ];


%% Outputs

O_simulator = [e_zs;
               Qdot(3);
               F_active_damper;
               O_model(1)]';



























































































end