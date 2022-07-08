function T = pose_to_matrix(pose)
    position = extract_position(pose);
    quaternion = extract_quaternion(pose);
    R = quat2rotm(quaternion);
    T = [   R,    position';
         0, 0, 0,         1];
end