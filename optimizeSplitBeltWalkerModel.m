function [b_offset_left, b_offset_right, lateral_push] ...
    = optimizeSplitBeltWalkerModel ...
      ( ...
        step_time_left, ...
        step_time_right, ...
        step_width_target, ...
        symmetry_mode, ...
        label ...
      )

    % set time
    T_total = 120;
    
    % initialize parameters
    b_offset_left_init = 0.01;
    b_offset_right_init = 0.01;
    
    % simulate once
    simulateSplitBeltWalkerModel(step_time_left, step_time_right, b_offset_left_init, b_offset_right_init, T_total, false);
    
    % optimize
    lateral_push = 0;
    if strcmp(symmetry_mode, 'symmetric')
        lower_bound = 0;
        upper_bound = [];
        b = fmincon(@errorFunctionSymmetric, b_offset_left_init, [], [], [], [], lower_bound, upper_bound);
        b_offset_left = b;
        b_offset_right = b;
    elseif strcmp(symmetry_mode, 'free')
        b_init = [b_offset_left_init, b_offset_right_init];
        lower_bound = [0 0];
        upper_bound = [];
        b = fmincon(@errorFunctionFree, b_init, [], [], [], [], lower_bound, upper_bound, @constraintFunctionLateralDeviation);
        b_offset_left = b(1);
        b_offset_right = b(2);
    elseif strcmp(symmetry_mode, 'symmetric_but_straight')
        lower_bound = 0;
        upper_bound = [];
        b = fmincon(@errorFunctionSymmetric, b_offset_left_init, [], [], [], [], lower_bound, upper_bound, @constraintFunctionLateralDeviation);
        b_offset_left = b;
        b_offset_right = b;
    elseif strcmp(symmetry_mode, 'symmetric_with_lateral_push')
        lower_bound = [];
        upper_bound = [];
        x_init = [b_offset_left_init, 0];
        x = fmincon(@errorFunctionSymmetricWithPush, x_init, [], [], [], [], lower_bound, upper_bound, @constraintFunctionLateralDeviation_withLateralPush);
        b_offset_left = x(1);
        b_offset_right = x(1);
        lateral_push = x(2);
    end
    
    % simulate result
    simulateSplitBeltWalkerModel(step_time_left, step_time_right, b_offset_left, b_offset_right, T_total, true, lateral_push, label);

% nested functions
function error = errorFunctionSymmetric(b)
    % use the same constant offset parameter value b for both sides to get symmetry
    step_width_actual = simulateSplitBeltWalkerModel(step_time_left, step_time_right, b, b, T_total, false);
    error = (step_width_actual - step_width_target)^2;
end

function error = errorFunctionFree(b)
    % allow different constant offset parameter values for each side
    b_left = b(1);
    b_right = b(2);
    step_width_actual = simulateSplitBeltWalkerModel(step_time_left, step_time_right, b_left, b_right, T_total, false);
    error = (step_width_actual - step_width_target)^2;
end

function error = errorFunctionSymmetricWithPush(x)
    % use the same constant offset parameter value b for both sides to get symmetry, but allow a lateral push
    b_here = x(1);
    lateral_push_here = x(2);
    step_width_actual = simulateSplitBeltWalkerModel(step_time_left, step_time_right, b_here, b_here, T_total, false, lateral_push_here);
    error = (step_width_actual - step_width_target)^2;
end
    
function [c, ceq] = constraintFunctionLateralDeviation(b)
    % constrain the overall lateral deviation to 0 -- this is for the model WITHOUT a lateral push
    if numel(b) == 1
        b_left = b;
        b_right = b;
    end
    if numel(b) == 2
        b_left = b(1);
        b_right = b(2);
    end
    
    [~, ~, ~, ~, overall_deviation] = simulateSplitBeltWalkerModel(step_time_left, step_time_right, b_left, b_right, T_total, false);
    c = [];
    ceq = overall_deviation;
end

function [c, ceq] = constraintFunctionLateralDeviation_withLateralPush(x)
    % constrain the overall lateral deviation to 0 -- this is for the model WITH a lateral push
    b_left = x(1);
    b_right = x(1);
    lateral_push_here = x(2);
    
    [~, ~, ~, ~, overall_deviation] = simulateSplitBeltWalkerModel(step_time_left, step_time_right, b_left, b_right, T_total, false, lateral_push_here);
    c = [];
    ceq = overall_deviation;
end    



end


