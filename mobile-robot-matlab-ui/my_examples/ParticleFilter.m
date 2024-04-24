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


% ===== Finish Here =================================================
% ===== Coursework 2 - Task 7 - PF Localization (State prediction) ==
% ===================================================================
    
%% Save the new particle poses
    set_property(obj, 'particles' , Particles);
%% Apply update step of Particle filter
    update(obj, omegas_map);

end

