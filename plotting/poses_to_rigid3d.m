function rigids = poses_to_rigid3d(poses)
    rigids = cell(length(poses), 1);
    for i = 1:length(poses)
        rigids{i} = pose_to_rigid3d(poses(i));
    end
end