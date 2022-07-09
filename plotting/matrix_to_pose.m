function pose = matrix_to_pose(T)
    pose = struct;
    pose.MessageType = 'geometry_msgs/Pose';
    t = T(1:3, 4)';
    R = T(1:3, 1:3);
    q = rotm2quat(R);
    pose.Position.X = t(1);
    pose.Position.Y = t(2);
    pose.Position.Z = t(3);
    pose.Orientation.W = q(1);
    pose.Orientation.X = q(2);
    pose.Orientation.Y = q(3);
    pose.Orientation.Z = q(4);
end