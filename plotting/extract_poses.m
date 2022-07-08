function poses = extract_poses(odometry_msgs)
    poses = cellfun(@(m) struct(m.Pose.Pose), odometry_msgs);
end