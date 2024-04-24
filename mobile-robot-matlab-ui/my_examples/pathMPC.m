function my_alg = pathMPC(my_alg, robot)
% -------------------------------------------------------------------------
if my_alg('is_first_time')
    %% =========================Initialization=====================
    % =====================Initial the variables here======================
    my_alg('path')=load('path_s.txt');% reference path (s shape)
    path=my_alg('path');              
    my_alg('dc_motor_signal_mode') = 'omega_setpoint';     % change if necessary to 'voltage_pwm'
    my_alg('right motor')   = 0;    % angular velocity set point for the right motor speed
    my_alg('left motor')    = 0;    % angular velocity set point for the left motor speed
    my_alg('t_loop')        = tic;  % operationg time for one loop
    my_alg('xr')=path(:,1); % reference path in x
    my_alg('yr')=path(:,2); % reference path in y
    my_alg('thetar')=path(:,3); % reference path in theta
    my_alg('path_x')=[]; % simulated robot path in x
    my_alg('path_y')=[]; % simulated robot in y
    my_alg('theta')=[];  % simulated robot in theta
    %% =======================Localizer============================
    my_alg('localizer') = LocalizationClass(...
        'method', 'wv',...
        'robot', robot,...
        'pose', [0 0 0],... % should always be the same as the initial 2D pose in mobile robot simulator GUI
        ...
        ...
        'ext_sensor_label', 'range');
    % =====================================================================
end

% Localization update
omegas_map = containers.Map({'right wheel', 'left wheel'},...
    [my_alg('right encoder'), my_alg('left encoder')]);
    my_alg('localizer') = Deadreckoning(my_alg('localizer'),omegas_map);
% Plotting section
my_alg = add_plot(my_alg, 'plot(my_alg(''localizer''))');     % plot pose estimation in the main figure
my_alg = add_plot(my_alg, 'plot(my_alg(''path_x''),my_alg(''path_y''),''k--'')');% plot robot passed path
my_alg('ref') = add_plot(my_alg, 'plot(my_alg(''xr''),my_alg(''yr''),''r'')');%plot premade reference path

%% Variables (add your own variables if needed) 
t_sampling          = 0.15;            % time period between two reference points (select a suitable time)
S                   = my_alg('localizer').pose; % current robot pose
w_r                 = my_alg('right motor');    % omega set point for right motor
w_l                 = my_alg('left motor');     % omega set point for left motor
L                   = 2*robot.components_tree.get('left motor').transformation(2,end);% length between wheels
r                   = robot.components_tree.get('left wheel').shape.diameter/2;% wheels radiu
my_alg('is_done')   = false;
%======================================================================
%======================================================================
%% build your system model and MPC controller in a control loop here
% =====================================================================



    
% =====================================================================
if my_alg('is_done')
%% make your plots here
figure(2)
%plot()
%figure(3) if needed

end


%% ====================update datas=========================================
my_alg('right motor')   = w_r;
my_alg('left motor')    = w_l;
my_alg('path_x')        = [my_alg('path_x') my_alg('localizer').pose(1)];
my_alg('path_y')        = [my_alg('path_y') my_alg('localizer').pose(2)];
my_alg('theta')         = [my_alg('theta')  my_alg('localizer').pose(3)];
return