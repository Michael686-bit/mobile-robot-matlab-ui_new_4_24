function my_alg = driveToGoal(my_alg, robot)

% Mohamed Mustafa, December 2020
% -------------------------------------------------------------------------

%% Initialise all variables and settings
if my_alg('is_first_time')
    % =========================Initialization==============================
    % =====================Initial the variables here====================== 
    
    % change if necessary to 'voltage_pwm'
    my_alg('dc_motor_signal_mode') = 'omega_setpoint';     
    % Initialise sequence to store estimated robot pose at each time step
    my_alg('path_x')=[];
    my_alg('path_y')=[];
    % initialise the right and left wheel angular velocity to 0
    my_alg('right motor')   = 0;
    my_alg('left motor')    = 0;
    
    %======================Set your goal here==============================
    my_alg('goal')= [8  0]'; %;[2 3]'
    % ===================================================================== 
    
    % =================== Set localisation alg ============================
    % =====================================================================
    % Select 1 for Deadreckoning and 2 for Particle Filter
    my_alg('localiser_type') = 1;  
    % =====================================================================
    % =====================================================================

    my_alg('e_I') = 0;
    % e_I = 0;
    

    % =====================================================================
    % =====================================================================

    
    % Based on the selected robot model define if the on-board sensor is
    % sonar/LiDAR/Laser
    if strfind(robot.description,'sonar')
        my_alg('Sensor')    = 1;
    elseif strfind(robot.description,'Lidar')
        my_alg('Sensor')    = 2;
    elseif strfind(robot.description,'laser')
        my_alg('Sensor')    = 3;
    end
    
    % Dead-reckoning
    if my_alg('localiser_type') == 1
        my_alg('localizer') = LocalizationClass(...
            'method', 'wv',...  % 'wv' is dead-reckoning localisation alg
            'robot', robot,...  % insert the robot object
            'pose', [0 0 0],... % Initial robot pose (must be the same as in GUI)
            ... % Not specified for dead-reckoning
            ... % Not specified for dead-reckoning
            'ext_sensor_label', 'range');   % specify sensor 'range' -> sonar/Laser
    elseif my_alg('localiser_type') == 2
    % Particle filter
        my_alg('localizer') = LocalizationClass(...
            'method', 'pf',... % 'pf' is particle filter localisation alg
            'robot', robot,... % insert the robot object
            'pose', [0 0 0],...% Initial robot pose (must be the same as in GUI)
            'n_particles', 30,...   % specify the number of particles
            'map', WorldClass('fname','world_0006.mat'),... % Specify the map
            'ext_sensor_label', 'lidar');   % specify sensor 'lidar' -> LiDAR
    end
end

%% Apply Dead reckoning or particle filter
% create an object that contains encoder reading for both right and left
% wheels
omegas_map = containers.Map({'right wheel', 'left wheel'},...
    [my_alg('right encoder'), my_alg('left encoder')]);
% Based on the localisation alg statement apply either dead-reckoning
% localisation or particle filter localisation
if strfind(my_alg('localizer').method,'wv')
    [my_alg('localizer'),delta_t]= Deadreckoning(my_alg('localizer'),omegas_map);
elseif strfind(my_alg('localizer').method,'pf')
    my_alg('localizer') = ParticleFilter(my_alg('localizer'),omegas_map);
end
% delta_t


%% Call required state variables and system constants

% Varaibles that can be used for Coursework 1 - Task 3
% Robot pose estimation [(m) (m) (rad)]'
S                   = my_alg('localizer').pose;
% Goal point (m)
goal                = my_alg('goal');
% right wheel angular velocity (rad/s)
w_r                 = omegas_map('right wheel');
% Left wheel angular velocity (rad/s)
w_l                 = omegas_map('left wheel'); 
% From the selected robot model call the distance between the two wheels(m)
l                   = 2*robot.components_tree.get('left motor').transformation(2,end);
% From the selected robot model call the radius of the wheel (m)
r                   = robot.components_tree.get('left wheel').shape.diameter/2;
% maximum angular velocity of the motor (rad/s)
w_sat               = 13.7;

e_I = my_alg('e_I');
% Boolen varaiable that indicates the end of simulation (true/false)
% (finish simulation/continue simulation)
my_alg('is_done')   = false;




% =====================================================================
%% ==== Coursework 1 - Task 3 - Motion Control (Part 1) ===============
% ===== Start Here ====================================================
% K_w = 5;
% k_turn = 50;

e_x = goal(1) - S(1);
e_y = goal(2) - S(2);

e_I = e_I + sqrt(e_x^2 + e_y^2);
my_alg('e_I') = e_I;

e_theata = atan2(e_y,e_x) - S(3);

% K_w = 20;
% k_turn = 450;
K_w = 10;
k_turn = 50;
K_I = 0.01;

w_l =  (K_w) * sqrt(e_x^2 + e_y^2)+ K_I * e_I  - k_turn * e_theata;%^2
w_r =  (K_w) * sqrt(e_x^2 + e_y^2)+ K_I * e_I  + k_turn * e_theata;%^2

% vr = (w_r+w_l)*r/2;
% wr = (w_r-w_l)*r/l;

if w_l>=w_sat
    w_l = w_sat;
end

if w_r>=w_sat
    w_r = w_sat;
end


if sqrt(e_x^2 + e_y^2)<0.1  %renew goal point
    my_alg('is_done');
    w_l = 0;
    w_r = 0;
end



% ===== Finish Here ===================================================
% ===== Coursework 1 - Task 3 - Motion Control (Part 1) ===============
% =====================================================================
%% Aquire robot input
my_alg('right motor')   = w_r;
my_alg('left motor')    = w_l;

%% Apply obstacle avoidance
if my_alg('Sensor') == 2
    my_alg = ObstacleAvoidanceLiDAR(my_alg, robot,delta_t);
elseif my_alg('Sensor') == 1 || my_alg('Sensor') == 3
    my_alg = ObstacleAvoidanceRange(my_alg, robot,delta_t);
end

%% Display results in GUI
% update the path sequence
my_alg('path_x')=[my_alg('path_x') my_alg('localizer').pose(1)];
my_alg('path_y')=[my_alg('path_y') my_alg('localizer').pose(2)];
% plot actual pose estimation in the main figure
my_alg = add_plot(my_alg, 'plot(my_alg(''localizer''))');
% plot pose estimation in the main figure
my_alg = add_plot(my_alg, 'plot(my_alg(''path_x''),my_alg(''path_y''),''k--'')');
% plot Robot covariance circle
my_alg = add_plot(my_alg, 'plot(my_alg(''obj,cov''))');

return