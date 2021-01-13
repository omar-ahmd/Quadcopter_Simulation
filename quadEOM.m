function sdot = quadEOM(t, s, qn, controlhandle, trajhandle, params,Data,speed,opt)


% convert state to quad stuct for control
qd{qn} = stateToQd(s);

% Get desired_state
desired_state = trajhandle(t, qn,Data,speed,opt);

% The desired_state is set in the trajectory generator
qd{qn}.pos_des      = desired_state.pos;
qd{qn}.vel_des      = desired_state.vel;
qd{qn}.acc_des      = desired_state.acc;
qd{qn}.yaw_des      = desired_state.yaw;
qd{qn}.yawdot_des   = desired_state.yawdot;

% get control outputs
[F, M, trpy, drpy] = controlhandle(qd, t, qn, params);

% compute derivative
sdot = quadEOM_readonly(t, s, F, M, params);

end
