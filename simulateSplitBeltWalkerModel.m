function ...
  [ ...
    step_width, ...
    x_trajectory, ...
    p_trajectory, ...
    time, ...
    overall_deviation, ...
    step_from_com_start_left, ...
    step_from_com_start_right, ...
    step_from_com_end_left, ...
    step_from_com_end_right, ...
    step_placement_left, ...
    step_placement_right, ...
    integrated_cop_from_com_left, ...
    integrated_cop_from_com_right, ...
    step_from_com_mean_left, ...
    step_from_com_mean_right, ...
    left_step_time_indices, ...
    right_step_time_indices ...
  ] = ...
    simulateSplitBeltWalkerModel(T_step_left, T_step_right, b_offset_left, b_offset_right, T_total, visualize_trajectories, lateral_push, label)
    
    % set default parameters
    if nargin < 8
        label = '';
    else
        label = [char(label) ': '];
    end
    
    if nargin < 7
        lateral_push = 0;
    end
    
    if nargin == 0
        T_step_left = 0.5;
        T_step_right = 0.5;
        b_offset_left = 0.01;
        b_offset_right = 0.01;
        T_total = 180;
        visualize_trajectories = 1;
    end
    k_proportional = 0;

    % set parameters
    number_of_dofs = 3;
    dt = 0.001;
    time = dt : dt : T_total;
    number_of_time_steps = length(time);

    % noise
    sigma_step = 0e-3;         % noise for foot placement during stepping
    rngSeed = 0;
    stream = RandStream('mt19937ar');
    reset(stream, rngSeed);
    RandStream.setGlobalStream(stream)

    z_c = 0.814;
    g = 9.81;
    omega = sqrt(g/z_c);

    % initialize system
    A_system = [0 1 0; 0 0 1; 0 omega^2 0];
    B_system = [0; 0; -omega^2];
    C_system = [1 0 -1/omega^2];
    A_Kalman = eye(3) + dt * A_system;
    B_kalman = dt * B_system;

    T_n = T_step_right;
    t_n = T_step_left;
    p_n = 0;
    x_n = 0;
    step_width_n = 0;
    foot_placement_n = 0;
    com_from_cop_at_step_end_n = 0;
    com_vel_at_step_end_n = 0;
    step_from_com_start_n = 0;
    step_from_com_end_n = 0;
    step_placement_n = 0;
    
    integrated_cop_from_com_n = 0;
    step_from_com_mean_n = 0;
    
    n = 1;
    step_number_trajectory = zeros(1, number_of_time_steps);
    step_number_trajectory(1) = n;

    x_trajectory = zeros(number_of_dofs, number_of_time_steps);
    p_trajectory = zeros(1, number_of_time_steps);
    xi_trajectory = zeros(1, number_of_time_steps);
    x_trajectory(1, 1) = 0;
    left_step_indices = 1;
    right_step_indices = [];
    step_time_indices = 1;

    for i_time = 2 : number_of_time_steps
        % time
        t = time(i_time);

        % lateral force
        w = B_kalman * lateral_push;
        
        % estimate
        x_estimate = x_trajectory(:, i_time-1);

        % iterate real system
        x_trajectory(:, i_time) = A_Kalman * x_trajectory(:, i_time-1) + w; % discrete version
        xi_trajectory(i_time) = x_trajectory(1, i_time) + x_trajectory(2, i_time)/omega;

        % output
        p_trajectory(i_time) = C_system * x_trajectory(:, i_time);

        % step control
        if t >= t_n(n)
            % step finished, calculate new step
            n = n+1;

            xi_hat = x_estimate(1) + 1/omega * x_estimate(2);

            % constant offset control
            if mod(n,2) 
                % n is odd - left step
                b_offset = b_offset_left;
                
            else
                % n is even - right step
                b_offset = b_offset_right;
            end
            
            com = x_estimate(1);
            x_reference = 0;
            p_old = p_trajectory(i_time);
            p_new = xi_hat - b_offset * (-1)^n + k_proportional * (xi_hat - x_reference);
            
            p_new = p_new + sigma_step * randn(1);
            a_new = omega^2*(x_trajectory(1, i_time) - p_new);
            step_from_com_start_n(n+1) = p_new - com; %#ok<AGROW>
            step_from_com_end_n(n) = p_trajectory(i_time) - com; %#ok<AGROW>
            com_from_cop_at_step_end_n(n) = com - p_trajectory(i_time); %#ok<AGROW>
            com_vel_at_step_end_n(n) = x_estimate(2); %#ok<AGROW>
            
            step_placement_n(n) = p_new - p_old; %#ok<AGROW>
            
            step_time_indices = [step_time_indices; i_time]; %#ok<AGROW>
            
            % integrate com-cop over previous step
            time_indices_this_step = step_time_indices(end-1) + 1 : i_time;
            time_this_step = time(time_indices_this_step);
            com_this_step = x_trajectory(1, time_indices_this_step);
            cop_this_step = p_trajectory(1, time_indices_this_step);
            com_from_cop_this_step = cop_this_step - com_this_step;
            cop_from_com_integral = cumtrapz(time_this_step, com_from_cop_this_step);
            integrated_cop_from_com_n(n) = cop_from_com_integral(end); %#ok<AGROW>
            
            % calculate step from com mean
            step = cop_this_step(1);
            step_from_com = step - com;
            step_from_com_mean_n(n) = mean(step_from_com); %#ok<AGROW>

            if mod(n,2) 
                % n is odd - left step
                T_n(n) = T_step_right;
                t_n(n) = t + T_step_left;
                left_step_indices = [left_step_indices n]; %#ok<AGROW>
            else
                % n is even - right step
                T_n(n) = T_step_left;
                t_n(n) = t + T_step_right;
                right_step_indices = [right_step_indices n]; %#ok<AGROW>
            end
    
            % update system
            x_trajectory(3, i_time) = a_new;
            p_n(n) = p_new; %#ok<AGROW>
            x_n(n) = x_trajectory(1, i_time); %#ok<AGROW>
            step_width_n(n) = abs(p_n(n) - p_n(n-1)); %#ok<AGROW>
            foot_placement_n(n) = p_n(n) - p_n(n-1); %#ok<AGROW>
        end
        step_number_trajectory(i_time) = n;

    end
    
    % extract data
    relaxation_time = 20;
    number_of_relaxation_steps = sum(t_n < relaxation_time);
    
    left_step_indices_after_relaxation = left_step_indices(left_step_indices>number_of_relaxation_steps);
    right_step_indices_after_relaxation = right_step_indices(right_step_indices>number_of_relaxation_steps);
    step_width = mean(step_width_n((number_of_relaxation_steps+1):end));
    step_from_com_start_left = mean(step_from_com_start_n(left_step_indices_after_relaxation));
    step_from_com_end_left = mean(step_from_com_end_n(left_step_indices_after_relaxation));
    step_from_com_start_right = mean(step_from_com_start_n(right_step_indices_after_relaxation));
    step_from_com_end_right = mean(step_from_com_end_n(right_step_indices_after_relaxation));
    
    step_placement_left = mean(step_placement_n(left_step_indices_after_relaxation));
    step_placement_right = mean(step_placement_n(right_step_indices_after_relaxation));
    
    integrated_cop_from_com_left = mean(integrated_cop_from_com_n(left_step_indices_after_relaxation));
    integrated_cop_from_com_right = mean(integrated_cop_from_com_n(right_step_indices_after_relaxation));
    
    step_from_com_mean_left = mean(step_from_com_mean_n(left_step_indices_after_relaxation));
    step_from_com_mean_right = mean(step_from_com_mean_n(right_step_indices_after_relaxation));
    
    overall_deviation = x_n(end) - x_n(number_of_relaxation_steps+1);
    
    left_step_time_indices = step_time_indices(left_step_indices);
    right_step_time_indices = step_time_indices(right_step_indices);
    
    % visualize
    if visualize_trajectories
        colors = lines(5);
        title_text = [label 'b left = ' num2str(b_offset_left) ', b right = ' num2str(b_offset_right) ', step width = ' num2str(step_width)];
        
        % plot lateral position
        set(0,'defaulttextinterpreter','latex')
        figure; subplot(3, 1, 1:2); hold on; title(title_text, 'fontsize', 16);
        plot(time, x_trajectory(1, :), 'linewidth', 4, 'DisplayName', 'CoM');
        plot(time, p_trajectory(1, :), ':', 'linewidth', 2, 'DisplayName', 'CoP');
        plot(time(step_time_indices), p_trajectory(1, step_time_indices), 'v', 'linewidth', 2, 'DisplayName', 'steps', 'MarkerEdgeColor', 'none', 'MarkerFaceColor', colors(5, :), 'markersize', 8);
        
        % add labels
        legend('Location', 'best');
        xlabel('time (s)', 'fontsize', 16)
        ylabel('lateral position', 'fontsize', 16)

        % plot steps
        subplot(3, 1, 3); hold on;
        plot(1:n, step_width_n, 'd', 'displayname', 'step width', 'markersize', 12, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', colors(2, :));
        plot(left_step_indices_after_relaxation, step_placement_n(left_step_indices_after_relaxation), 'v', 'displayname', 'step placement left', 'markersize', 10, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', colors(3, :));
        plot(right_step_indices_after_relaxation, step_placement_n(right_step_indices_after_relaxation), '^', 'displayname', 'step placement right', 'markersize', 10, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', colors(4, :));
        
        % add labels
        legend('Location', 'best');
        xlabel('steps', 'fontsize', 16)
        ylabel('discrete variables', 'fontsize', 16)
        ylim([min(step_placement_n) - 0.02, max(step_placement_n) + 0.02]);

    end    
end
