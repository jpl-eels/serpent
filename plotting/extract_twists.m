function twists = extract_twists(odometry_msgs)
    twists = cellfun(@(m) struct(m.Twist.Twist), odometry_msgs);
end