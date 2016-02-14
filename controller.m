function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Gains
Kd=[1; 1; 1];
Kp=[100; 100; 800];
Kd_ang=[1; 1; 1];
Kp_ang=[160; 160; 160];



%3. Change only kph to 800. runsim- now the quad is going straight up

%4. Now change kppsi- to 160. runsim- now the quad is going up and then turn the wrong way.

%5. Now change both kptheta, and kph, to 160. runsim- now the quad is moving sort of to the right place...

%6. continue from there on your own :)



grav=params.gravity;
mass=params.mass;

% Compute command accelerations
cmd_accel=des_state.acc + ...
    Kd.*(des_state.vel-state.vel) + ...
    Kp.*(des_state.pos-state.pos);

% Thrust (force to overcome gravity plus achieve cmd_accel in z)
F=mass*(grav + cmd_accel(3));

% test for limits and force them
if F<params.minF
    F=params.minF;
end
if F>params.maxF
    F=params.maxF;
end

% compute desired roll and pitch
phi_des = (1/grav)*(cmd_accel(1)*sin(des_state.yaw) - ...
    cmd_accel(2)*cos(des_state.yaw));
theta_des = (1/grav)*(cmd_accel(1)*cos(des_state.yaw) + ...
    cmd_accel(2)*sin(des_state.yaw));

% build desired angle vector and angle rate vector
rot_des = [phi_des; theta_des; des_state.yaw];
omega_des = [0; 0; des_state.yawdot];

% Moment
M = Kp_ang.*(rot_des-state.rot) + ...
    Kd_ang.*(omega_des-state.omega);

% =================== Your code ends here ===================

end