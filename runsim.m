% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part

function [QP] = Simulate(h_3d,trajhandle,qn,Data,Speed,opt,params)


addpath('utils')
addpath('trajectories')
try
    init_script;
catch
end
% controller
controlhandle = @controller;

% real-time 
real_time = true;

% max time
time_tol = 25;



%% **************************** FIGURES *****************************
quadcolors = lines(qn);
%% *********************** INITIAL CONDITIONS ***********************

max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, qn,Data,Speed,opt);
des_stop  = trajhandle(inf, qn,Data,Speed,opt);
stop{qn}  = des_stop.pos;

x0{qn}    = init_state( des_start.pos, 0 );



x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
% Main loop
for iter = 1:max_iter

    

    timeint = time:tstep:time+cstep;
    if iter == 1
        QP{qn} = QuadPlot(qn, x0{qn}, params.arm_length, 0.046, quadcolors(qn,:), max_iter, h_3d);
        desired_state = trajhandle(time, qn, Data,Speed,opt);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time); 
    end
        
        
    FV = @(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params, Data,Speed,opt);
    % Run simulation
    [tsave, xsave] = ode45(FV , timeint, x{qn});
    x{qn}    = xsave(end, :)';


    % Update quad plot
    t =time + cstep;
    desired_state = trajhandle(t, qn,Data,Speed,opt);
    
    hold(h_3d,'on')
    QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], t);
    hold(h_3d,'off')   

    time = time + cstep; % Update simulation time


    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol,qn)
        break
    end
end
fprintf('finished.\n')


