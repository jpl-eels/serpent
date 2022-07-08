function t = extract_positions(poses)
    % There must be a more efficient way
    t = zeros(length(poses), 3);
    t(:,1) = arrayfun(@(m) double(m.Position.X), poses);
    t(:,2) = arrayfun(@(m) double(m.Position.Y), poses);
    t(:,3) = arrayfun(@(m) double(m.Position.Z), poses);
end