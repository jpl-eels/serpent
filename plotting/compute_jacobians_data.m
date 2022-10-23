function jacobians = compute_jacobians_data(jacobian_data)
    jacobians = struct;
    jacobians.names = jacobian_data.names;
    jacobians.num_entries = length(jacobian_data.jacobians);
    for i = 1:jacobians.num_entries
        entry = struct;
        entry.jacobian_dim = 0;
        if ~isempty(jacobian_data.jacobians{i})
            entry.jacobian_dim = ...
                length(jacobian_data.jacobians{i}{1}.Data);
        end
        entry.jacobians = zeros(length(jacobian_data.jacobians{i}), ...
            entry.jacobian_dim);
        for j = 1:length(jacobian_data.jacobians{i})
            entry.jacobians(j, :) = jacobian_data.jacobians{i}{j}.Data;
        end
        entry.squared_jacobians = entry.jacobians.^2;
        jacobians.entries{i} = entry;
    end
end