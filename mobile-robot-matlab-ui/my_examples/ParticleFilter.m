function [obj] = ParticleFilter(obj,omegas_map)

%% Call the required state varaibles and system constants
% From the selected robot model call the error associated with computing the angular velocity for each wheel
    k               = obj.robot.wheel_error_constant;
% From the selected robot model call the distance between the two wheels(m)
    L               = 2*obj.robot.components_tree.get('left motor').transformation(2,end);
% From the selected robot model call the radius of the wheel (m)
    r               = obj.robot.components_tree.get('left wheel').shape.diameter/2;
% The number of particles
    m               = obj.n_particles;
% The particles poses
    Particles       = obj.particles;
% The encoder reading of the right wheel angular velocity (rad/s)
    w_r             = omegas_map('right wheel');
% The encoder reading of the left wheel angular velocity (rad/s)
    w_l             = omegas_map('left wheel');
% The time step
    dt              = toc(obj.timestamp);               % Compute Change in time
    set_property(obj, 'timestamp' , tic);               % reset stopwatch
    
% ===================================================================
%% ==== Coursework 2 - Task 7 - PF Localization (State prediction) ==
% ===== Start Here ==================================================                


%Write your code here


% Write your code here for state prediction
% Predict the new pose of each particle based on the motion model
% for i = 1:m
%     % Compute odometry-based motion model
%     delta_s = 0.5 * r * (w_r(i) + w_l(i)) * dt;  % Compute the change in distance
%     delta_theta = (r / L) * (w_r(i) - w_l(i)) * dt;  % Compute the change in heading angle
% 
%     % Update particle pose
%     theta = Particles(i, 3) + delta_theta;  % Update heading angle
%     x = Particles(i, 1) + delta_s * cos(theta);  % Update x position
%     y = Particles(i, 2) + delta_s * sin(theta);  % Update y position
% 
%     % Update particle pose in the Particles matrix
%     Particles(i, :) = [x, y, theta];
% end


% 计算每个轮子的线速度


v_r = r * w_r;  % 右轮线速度
v_l = r * w_l;  % 左轮线速度

% 计算机器人的总体线速度v和角速度omega
v = (v_r + v_l) / 2;          % 平均线速度
omega = (v_r - v_l) / L;      % 角速度
% 
% 初始化噪声的协方差矩阵
sigma = [k^2 0 0; 0 k^2 0; 0 0 (k/L)^2];
% 
% 遍历所有粒子，更新它们的位置和方向
for i = 1:m

    % 当前粒子的状态
    x = Particles(1, i);
    y = Particles(2, i);
    theta = Particles(3, i);
    % disp(x)
    % disp(y)
    % disp(theta)
    % 根据运动模型预测新的状态
    x_new = x + v * cos(theta) * dt;
    y_new = y + v * sin(theta) * dt;
    theta_new = theta + omega * dt;
    % disp(x_new)
    % disp(y_new)
    % disp(theta_new)
    % 加入噪声
    noise = mvnrnd([0 0 0], sigma);
    x_new = x_new + noise(1);
    y_new = y_new + noise(2);
    theta_new = theta_new + noise(3);
    disp(x_new)
    disp(y_new)
    disp(theta_new) 
    % 更新粒子
    Particles(1, i) = x_new;
    Particles(2, i) = y_new;
    Particles(3, i) = theta_new;

end

% ===== Finish Here =================================================
% ===== Coursework 2 - Task 7 - PF Localization (State prediction) ==
% ===================================================================
    
%% Save the new particle poses
    set_property(obj, 'particles' , Particles);
%% Apply update step of Particle filter
    update(obj, omegas_map);

end

