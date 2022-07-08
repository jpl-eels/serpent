function angular_velocities = extract_angular_velocities(twists)
    % There must be a more efficient way
    angular_velocities = zeros(length(twists), 3);
    angular_velocities(:,1) = arrayfun(@(m) double(m.Angular.X), twists);
    angular_velocities(:,2) = arrayfun(@(m) double(m.Angular.Y), twists);
    angular_velocities(:,3) = arrayfun(@(m) double(m.Angular.Z), twists);
end