function q = extract_quaternions(poses)
    % There must be a more efficient way
    q = zeros(length(poses), 4);
    q(:,1) = arrayfun(@(m) double(m.Orientation.W), poses);
    q(:,2) = arrayfun(@(m) double(m.Orientation.X), poses);
    q(:,3) = arrayfun(@(m) double(m.Orientation.Y), poses);
    q(:,4) = arrayfun(@(m) double(m.Orientation.Z), poses);
end