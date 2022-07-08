function linear_velocities = extract_linear_velocities(twists)
    % There must be a more efficient way
    linear_velocities = zeros(length(twists), 3);
    linear_velocities(:,1) = arrayfun(@(m) double(m.Linear.X), twists);
    linear_velocities(:,2) = arrayfun(@(m) double(m.Linear.Y), twists);
    linear_velocities(:,3) = arrayfun(@(m) double(m.Linear.Z), twists);
end