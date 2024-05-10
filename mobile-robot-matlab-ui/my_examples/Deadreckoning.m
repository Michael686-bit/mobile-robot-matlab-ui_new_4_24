function [obj,dt] = Deadreckoning(obj,omegas_map)
%Deadreckoning Summary of this function goes here
%   Detailed explanation goes here

%% Call required state variables and system constants
    % Variables that can be used for Coursework 1 - Task 2
    % From the selected robot model call the error associated with
    % computing the angular velocity for both wheels
    kr              = obj.robot.wheel_error_constant; %Ignore for CW1
    kl              = kr;
    % From the selected robot model call the radius of the wheel (m)
    r               = obj.robot.components_tree.get('left wheel').shape.diameter/2;
    % From the selected robot model call the distance between the two wheels(m)
    l               = 2*obj.robot.components_tree.get('left motor').transformation(2,end);
    % The encoder reading of the right wheel angular velocity (rad/s)
    w_r             = omegas_map('right wheel');
    % The encoder reading of the left wheel angular velocity (rad/s)
    w_l             = omegas_map('left wheel');
    % Robot pose mean
    S               = obj.pose;
    % Robot pose covariance
    Sig             = obj.cov ;   %* 0; %Ignore for CW1
    % The time step
    % global dt
    dt              = toc(obj.timestamp) ;              % Compute Change in time
    set_property(obj, 'timestamp' , tic);               % reset stopwatch
    
    % ====================================================================
    %% ==== Coursework 1 - Task 2 - Dead Reckoning Localization ===========
    % ===== Start Here ====================================================
    delta_d = (dt * w_l * r + dt * w_r * r)/2;
    delta_theata = (dt * w_r * r - dt * w_l * r) / l;
    v = (w_l * r +  w_r * r)/2;

    S(1) = S(1) + delta_d * cos(S(3));
    S(2) = S(2) + delta_d * sin(S(3));
    S(3) = S(3) + delta_theata;
    

    H_k = [1 0 -dt * v * sin(S(3));
           0 1  dt * v * cos(S(3));
           0 0    1        ];

    delta_wk = (0.5 * r * dt ).*[ cos(S(3)) cos(S(3));
                                cos(S(3)) cos(S(3));
                                2/l    2/l];
    Q_k = delta_wk * [kr * abs(w_r)    0;
                        0            kl * abs(w_l)] * delta_wk';
    

    Sig = H_k * Sig * H_k' + Q_k;


    % S(3) = mod(S(3) + pi, 2*pi) - pi;
                
    % ===== Finish Here ===================================================
    % ===== Coursework 1 -  Task 2 - Dead Reckoning Localization ===========
    % =====================================================================
%% Save Robot pose and covariance
    set_property(obj, 'pose' , S);
    set_property(obj, 'cov' , Sig);
end

