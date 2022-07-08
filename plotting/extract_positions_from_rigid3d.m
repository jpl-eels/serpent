function t = extract_positions_from_rigid3d(poses)
    % There must be a more efficient way
    t = zeros(length(poses), 3);
    t(:,1) = cellfun(@(m) double(m.Translation(1)), poses);
    t(:,2) = cellfun(@(m) double(m.Translation(2)), poses);
    t(:,3) = cellfun(@(m) double(m.Translation(3)), poses);
end