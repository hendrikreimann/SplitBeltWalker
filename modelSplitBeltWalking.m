% flags
optimize_parameters         = 1;
use_hard_coded_parameters   = 0;
show_figure                 = 1;
save_figure                 = 0;

% define step time and step width target parameters -- each entry specifies one model
step_time_tied_slow = 0.68;
step_time_tied_fast = 0.5;
step_time_split_slow = 0.61;
step_time_split_fast = 0.51;
step_width_slow = 0.145;
step_width_fast = 0.135;
step_width_split = 0.145;

model_specification_table = table ...
  ( ...
    'Size', [0, 5], ...
    'VariableTypes', {'double', 'double', 'double', 'string', 'string'}, ...
    'VariableNames', {'step_time_left', 'step_time_right', 'step_width_target', 'mode', 'label'} ...
  );
model_specification_table = [model_specification_table; {step_time_tied_slow, step_time_tied_slow, step_width_slow, 'symmetric', 'tied slow'}];
model_specification_table = [model_specification_table; {step_time_tied_fast, step_time_tied_fast, step_width_fast, 'symmetric', 'tied fast'}];

model_specification_table = [model_specification_table; {step_time_split_slow, step_time_split_fast, step_width_split, 'free', 'split young'}];
model_specification_table = [model_specification_table; {step_time_split_slow, step_time_split_fast, step_width_split, 'symmetric_but_straight', 'split old without CoP shift'}];
model_specification_table = [model_specification_table; {step_time_split_slow, step_time_split_fast, step_width_split, 'symmetric_with_lateral_push', 'split old'}];
number_of_models = size(model_specification_table, 1);

% determine model parameters
if optimize_parameters
    model_parameter_table = table ...
      ( ...
        'Size', [0, 3], ...
        'VariableTypes', {'double', 'double', 'double'}, ...
        'VariableNames', {'b_offset_left', 'b_offset_right', 'lateral_push'} ...
      );
    for i_model = 1 : number_of_models
        % optimize model parameters to specification
        [b_offset_left, b_offset_right, lateral_push] = optimizeSplitBeltWalkerModel ...
          ( ...
            model_specification_table.step_time_left(i_model), ...
            model_specification_table.step_time_right(i_model), ...
            model_specification_table.step_width_target(i_model), ...
            model_specification_table.mode(i_model), ...
            model_specification_table.label(i_model) ...
          );
      
        % store optimized parameters
        model_parameter_table = [model_parameter_table; {b_offset_left, b_offset_right, lateral_push}];
    end

end

if use_hard_coded_parameters
    % these are the results of the optimization, hard-coded for speed
    model_parameter_table = cell2table ...
      ( ...
        { ...
          0.0129955655899671,0.0129955655899671,0; ...
          0.0206785534824837,0.0206785534824837,0; ...
          0.0152739427447065,0.0226331650847485,0; ...
          8.00977541506368e-09,8.00977541506368e-09,0; ...
          0.0190978300572378,0.0190978300572378,-0.0188194636672007 ...
        }, ...
        'VariableNames', {'b_offset_left', 'b_offset_right', 'lateral_push'} ...
      );
end

% simulate models
total_time = 30;
x_trajectory_cell = cell(1, number_of_models);
p_trajectory_cell = cell(1, number_of_models);
step_width = zeros(1, number_of_models);
step_from_com_start_left = zeros(1, number_of_models);
step_from_com_end_left = zeros(1, number_of_models);
step_from_com_start_right = zeros(1, number_of_models);
step_from_com_end_right = zeros(1, number_of_models);
integrated_cop_from_com_left = zeros(1, number_of_models);
integrated_cop_from_com_right = zeros(1, number_of_models);
step_from_com_mean_left = zeros(1, number_of_models);
step_from_com_mean_right = zeros(1, number_of_models);
left_step_time_indices = cell(1, number_of_models);
right_step_time_indices = cell(1, number_of_models);
step_placement_left = zeros(1, number_of_models);
step_placement_right = zeros(1, number_of_models);
for i_model = 1 : number_of_models
    % simulate
    [ ...
      step_width_this_value, ...
      x_trajectory, ...
      p_trajectory, ...
      time, ...
      ~, ...
      step_from_com_start_left_this_value, ...
      step_from_com_start_right_this_value, ...
      step_from_com_end_left_this_value, ...
      step_from_com_end_right_this_value, ...
      step_placement_left_this_value, ...
      step_placement_right_this_value, ...
      integrated_cop_from_com_left_this_value, ...
      integrated_cop_from_com_right_this_value, ...
      step_from_com_mean_left_this_value, ...
      step_from_com_mean_right_this_value, ...
      left_step_time_indices_this_value, ...
      right_step_time_indices_this_value ...
    ] = ...
    simulateSplitBeltWalkerModel ...
      ( ...
        model_specification_table.step_time_left(i_model), ...
        model_specification_table.step_time_right(i_model), ...
        model_parameter_table.b_offset_left(i_model), ...
        model_parameter_table.b_offset_right(i_model), ...
        total_time, ...
        false, ...
        model_parameter_table.lateral_push(i_model) ...
      );
    
    % store results
    step_width(i_model) = step_width_this_value;
    x_trajectory_cell{i_model} = x_trajectory;
    p_trajectory_cell{i_model} = p_trajectory;
    step_from_com_start_left(i_model) = step_from_com_start_left_this_value;
    step_from_com_end_left(i_model) = step_from_com_end_left_this_value;
    step_from_com_start_right(i_model) = step_from_com_start_right_this_value;
    step_from_com_end_right(i_model) = step_from_com_end_right_this_value;
    step_placement_left(i_model) = step_placement_left_this_value;
    step_placement_right(i_model) = step_placement_right_this_value;
    integrated_cop_from_com_left(i_model) = integrated_cop_from_com_left_this_value;
    integrated_cop_from_com_right(i_model) = integrated_cop_from_com_right_this_value;
    step_from_com_mean_left(i_model) = step_from_com_mean_left_this_value;
    step_from_com_mean_right(i_model) = step_from_com_mean_right_this_value;
    left_step_time_indices{i_model} = left_step_time_indices_this_value;
    right_step_time_indices{i_model} = right_step_time_indices_this_value;
end

% visualize
if show_figure
    % define which models to show
    models_to_visualize = [5, 4, 3, 2, 1];
    
    % set visualization parameters
    number_of_models_to_visualize = length(models_to_visualize);
    colors = flip(lines(number_of_models_to_visualize), 1);
    x_offsets = [0, 0.22, 0.44, 0.66, 0.88];
    time_to_show = 5;           % width of the time window that will be shown
    relaxation_time = 10;       % start of the time window that will be shown
    [~, number_of_time_indices_to_show] = min(abs(time - time_to_show));
    grey = [0.5 0.5 0.5];
    
    % plot
    figure; hold on;
    for i_model = 1 : number_of_models_to_visualize
        model_index = models_to_visualize(i_model);
        this_x_trajectory = x_trajectory_cell{model_index};
        this_p_trajectory = p_trajectory_cell{model_index};
        this_left_step_time_indices = left_step_time_indices{model_index};
        this_right_step_time_indices = right_step_time_indices{model_index};
        
        % create step position timeseries for each foot from contact point
        this_left_step = zeros(size(time)) * NaN;
        for i_step = 1 : length(this_left_step_time_indices)
            this_step_start_index = this_left_step_time_indices(i_step) + 1;
            if this_step_start_index < length(time)
                this_step_end_index = min(this_right_step_time_indices(this_right_step_time_indices > this_step_start_index));
                this_step_x_pos = this_p_trajectory(this_step_start_index);
                this_left_step(this_step_start_index : this_step_end_index) = this_step_x_pos;
            end
        end
        this_right_step = zeros(size(time)) * NaN;
        for i_step = 1 : length(this_right_step_time_indices)
            this_step_start_index = this_right_step_time_indices(i_step) + 1;
            this_step_end_index = min(this_left_step_time_indices(this_left_step_time_indices > this_step_start_index));
            this_step_x_pos = this_p_trajectory(this_step_start_index);
            this_right_step(this_step_start_index : this_step_end_index) = this_step_x_pos;
        end
        
        % find start index as first left heelstrike after relaxation time
        left_heelstrike_times = time(this_left_step_time_indices);
        first_left_heelstrike_after_relaxation_index = find(left_heelstrike_times > relaxation_time, 1, 'first');
        start_time_index = this_left_step_time_indices(first_left_heelstrike_after_relaxation_index) + 1;
        end_time_index = start_time_index + number_of_time_indices_to_show;
        
        % define time window
        window = start_time_index+1 : end_time_index;
        time_window = time(1 : number_of_time_indices_to_show);
        this_x_pos_window = this_x_trajectory(1, window);
        this_p_pos_window = this_p_trajectory(1, window);
        this_left_step_window = this_left_step(1, window);
        this_right_step_window = this_right_step(1, window);
        this_left_heelstrike_indices_window = this_left_step_time_indices - start_time_index + 1;
        this_left_heelstrike_indices_window(this_left_heelstrike_indices_window < 1) = [];
        this_left_heelstrike_indices_window(this_left_heelstrike_indices_window > number_of_time_indices_to_show) = [];
        this_right_heelstrike_indices_window = this_right_step_time_indices - start_time_index + 1;
        this_right_heelstrike_indices_window(this_right_heelstrike_indices_window < 1) = [];
        this_right_heelstrike_indices_window(this_right_heelstrike_indices_window > number_of_time_indices_to_show) = [];
        
        % extract data from window
        this_left_step_positions = this_p_pos_window(this_left_heelstrike_indices_window);
        this_right_step_positions = this_p_pos_window(this_right_heelstrike_indices_window);
        this_number_of_steps_to_use = min([length(this_left_step_positions) length(this_right_step_positions)]);
        this_x_pos_center_all = (this_left_step_positions(1:this_number_of_steps_to_use) + this_right_step_positions(1:this_number_of_steps_to_use)) * 0.5;
        this_x_pos_center = mean(this_x_pos_center_all);
        
        % define handle visibility to add legend, but only for first model
        if i_model == 1
            handle_visibility = 'on';
        else
            handle_visibility = 'off';
        end
        
        % do the actual plots
        plot(time_window([1 end]), x_offsets(i_model)*[1 1], 'color', grey, 'linewidth', 2, 'HandleVisibility', 'off');
        
        plot ...
          ( ...
            time_window, this_x_pos_window - this_x_pos_center + x_offsets(i_model), ...
            'linewidth', 4, ...
            'HandleVisibility', handle_visibility, ...
            'color', colors(i_model, :) ...
          );
        
        plot ...
          ( ...
            time_window, this_p_pos_window - this_x_pos_center + x_offsets(i_model), ...
            'linewidth', 1, ...
            'HandleVisibility', handle_visibility, ...
            'color', colors(i_model, :) ...
          );
        
        plot ...
          ( ...
            time_window(this_left_heelstrike_indices_window), ...
            this_p_pos_window(this_left_heelstrike_indices_window) - this_x_pos_center + x_offsets(i_model), ...
            '>', ...
            'MarkerSize', 10, ...
            'MarkerFaceColor', colors(i_model, :), ...
            'MarkerEdgeColor', 'none' ...
          )
        plot ...
          ( ...
            time_window(this_right_heelstrike_indices_window), ...
            this_p_pos_window(this_right_heelstrike_indices_window) - this_x_pos_center + x_offsets(i_model), ...
            '>', ...
            'MarkerSize', 10, ...
            'MarkerFaceColor', colors(i_model, :), ...
            'MarkerEdgeColor', 'none' ...
          )
    end
    
    % groom ticks
    yticks_single = [-0.1 -0.05 0 0.05 0.1];
    yticks = repmat(yticks_single, 1, number_of_models_to_visualize) + reshape(repmat(x_offsets(1:number_of_models_to_visualize), numel(yticks_single), 1), 1, []);
    yticklabels = cell(size(yticks));
    for i_model = 1 : number_of_models_to_visualize
        yticklabels{numel(yticks_single)*(i_model-1) + 1} = '-0.1';
        yticklabels{numel(yticks_single)*(i_model-1) + 2} = '-0.05';
        yticklabels{numel(yticks_single)*(i_model-1) + 3} = model_specification_table.label(models_to_visualize(i_model));
        yticklabels{numel(yticks_single)*(i_model-1) + 4} = '0.05';
        yticklabels{numel(yticks_single)*(i_model-1) + 5} = '0.1';
    end
    set(gca, 'ytick', yticks, 'YTickLabel', yticklabels)
    set(gca, 'ylim', [yticks(1) yticks(end)])
    
    % legend
    legend('CoM', 'CoP', 'foot placement')

    % save figure
    if save_figure
        figure_size = [500 400];
        set(gcf, 'Position', [100 100 figure_size])
        set(gcf, 'PaperUnits', 'points');
        set(gcf, 'PaperSize', figure_size);

        saveas(gcf, 'modelFigureRaw.pdf', 'pdf')

        set(get(gca, 'xaxis'), 'visible', 'off');
        set(get(gca, 'yaxis'), 'visible', 'off');
        set(get(gca, 'xlabel'), 'visible', 'off');
        set(get(gca, 'ylabel'), 'visible', 'off');
        set(get(gca, 'title'), 'visible', 'off');
        set(gca, 'xticklabel', '');
        set(gca, 'yticklabel', '');
        set(gca, 'position', [0 0 1 1]);
        legend(gca, 'hide');
        print(gcf, 'modelFigureRaw_clear.pdf', '-dpdf')
        close(gcf)
    end    
end
    

