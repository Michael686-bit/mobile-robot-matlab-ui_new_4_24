function [my_alg] = ObstacleAvoidanceLiDAR_new(my_alg, robot)
%% Initialise all variables and settings
if my_alg('is_first_time') 
% =========================================================================
% ========= Coursework 2 - Task 4 - Obstacle Avoidance using LiDAR =========
% ========= Initialise variables Here =====================================
        
    % From the selected robot model access the angle Span of LiDAR
    Angle_span              = robot.components_tree.get('lidar').angle_span/360*2*pi;
    % From the selected robot model access the angle resolution of LiDAR
    Angle_resolution        = robot.components_tree.get('lidar').angle_resolution/360*2*pi;
    % create a vector of the set of all possible bearings of LiDAR in order
    my_alg('Th')            = -(Angle_span/2)+Angle_resolution:Angle_resolution:(Angle_span/2);
        
% ========= Finish initialisation Here ====================================
% ========= Coursework 2 - Task 4 - Obstacle Avoidance using LiDAR ======
% =========================================================================
end

%% Call required state variables and system constants
% Varaibles that can be used for Coursework 1 - Task 5
% Robot pose estimation [(m) (m) (rad)]'
S                   = my_alg('localizer').pose;
% From the selected robot model call the distance between the two wheels(m)
L                   = 2*robot.components_tree.get('left motor').transformation(2,end);
% From the selected robot model call the radius of the wheel (m)
r                   = robot.components_tree.get('left wheel').shape.diameter/2;
% maximum angular velocity of the motor (rad/s)
w_sat               = 13.8;
% right wheel angular velocity (rad/s)
w_r                 = my_alg('right motor');
% Left wheel angular velocity (rad/s)
w_l                 = my_alg('left motor');
% The set of all bearings
Theta               = my_alg('Th');
%
Range               = my_alg('lidar');

% =========================================================================
%% ======== Coursework 2 - Task 4 - Obstacle Avoidance - LiDAR ============
% ========= Start Here ====================================================
    

%Write your code here






% ========= Finish Here ===================================================
% ========= Coursework 2 - Task 4 - Obstacle Avoidance - LiDAR ============
% =========================================================================
    
%% Acquire robot input
    % Acquire wheels velocity
    my_alg('right motor')   = w_r;
    my_alg('left motor')    = w_l;

end

