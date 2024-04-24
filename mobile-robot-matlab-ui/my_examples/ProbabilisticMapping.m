function my_alg = ProbabilisticMapping(my_alg, robot)

%% Initialisation
    if my_alg('is_first_time')
        % signal form used for controlling the motor change if necessary to 'voltage_pwm'
        my_alg('dc_motor_signal_mode') = 'omega_setpoint';
        
        map.ll_corner = [-10 -5];   % left lower corner coordinate
        map.ur_corner = [10 5];     % upper right corner coordinate
        map.res = 0.1;              % grid map resolution
        % number of grids in for row and column
        map_size = floor(map.ur_corner - map.ll_corner)./map.res+1;
        % set each grid in the map to 50% probability
        map.OGrid = 0.5* ones(map_size(2),map_size(1));
        % the type of mapping algorithm 'binary' or 'log_odds'
        map.type = 'binary';
        % Save the object map into the main algorithm
        my_alg('map') = map;
        
        % From the selected robot model access the angle Span of LiDAR
        Angle_span              = robot.components_tree.get('lidar').angle_span/360*2*pi;
        % From the selected robot model access the angle resolution of LiDAR
        Angle_resolution        = robot.components_tree.get('lidar').angle_resolution/360*2*pi;
        % create a vector of the set of all possible bearings of LiDAR in order
        my_alg('Th')            = -(Angle_span/2)+Angle_resolution:Angle_resolution:(Angle_span/2);
    end
    
%% Use driveToGoal to move the robot (change to driveToMultipleGoals if necessary)
    my_alg = driveToGoal(my_alg, robot);

%% Required variables
    % Robot pose estimation [(m) (m) (rad)]'
    S                   = my_alg('localizer').pose;
    % Vector of the set of all possible bearings of LiDAR in order
    Theta               = my_alg('Th');
    % Vectoe of range measurements with respect to 'Theta'
    Range               = my_alg('lidar');
    % From the selected robot model access the maximum range measurement
    max_range           = robot.components_tree.get('lidar').max_range;
    % map object
    map                 = my_alg('map');

% =========================================================================
%% ==== Coursework 2 - Tasks 6 - Occupancy Grid Probabilistic Mapping =====
% ===== Start Here ========================================================



%Write youir code here



% ===== Finish Here =======================================================
% ===== Coursework 2 - Tasks 6 - Occupancy Grid Probabilistic Mapping =====
% =========================================================================    

    % save map object
    my_alg('map') = map;

    % plot the estimated map
    figure(2), clf
    worldPlot2( map,1);
    title('Binary Mapping');
    
return