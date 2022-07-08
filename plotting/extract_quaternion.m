function quat = extract_quaternion(pose)
    quat = [pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, ...
        pose.Orientation.Z];
end