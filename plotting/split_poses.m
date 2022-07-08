function [translation, rotation] = split_poses(poses)
    size = length(poses);
    translation = zeros(size, 3);
    rotation = zeros(size, 3);
    for i = 1:size
        pose = poses{i};
        translation(i, :) = pose(1:3, 4)';
        axang = rotm2axang(pose(1:3, 1:3));
        rotation(i, :) = axang(:, 1:3) * axang(:, 4);
    end
end