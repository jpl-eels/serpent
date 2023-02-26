function [position, orientation] = interp_transform(gt_time_1, ...
    gt_time_2, gt_pose_1, gt_pose_2, t_query)
    interp_coeff = (t_query - gt_time_1) / (gt_time_2 - gt_time_1);

    pos1 = extract_position(gt_pose_1);
    pos2 = extract_position(gt_pose_2);
    position = pos1 + (pos2 - pos1) * interp_coeff;

    q1 = extract_quaternion(gt_pose_1);
    q1 = q1 / norm(q1);
    q2 = extract_quaternion(gt_pose_2);
    q2 = q2 / norm(q2);
    if interp_coeff < 0 || interp_coeff > 1
        error("interp coefficient wasn't between 0 and 1");
    end
    orientation = quatinterp(q1, q2, interp_coeff, "slerp");
end