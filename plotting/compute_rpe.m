function [rpe_timestamps, rpes] = compute_rpe(gt_timestamps, ...
        gt_poses, timestamps, poses)
    timestamps = timestamps(timestamps <= gt_timestamps(end));
    rpe_timestamps = timestamps(2:end);
    size = length(rpe_timestamps);
    rpes = cell(size, 1);
    gt_index = 1;
    prev_gt_T = eye(4);
    prev_T = eye(4);
    for i = 1:(size + 1)
        t = timestamps(i);
        pose = poses(i);
        T = pose_to_matrix(pose);
        while gt_timestamps(gt_index) < t
            gt_index = gt_index + 1;
        end
        [gt_position, gt_orientation] = interp_transform(...
            gt_timestamps(gt_index - 1), gt_timestamps(gt_index), ...
            gt_poses(gt_index - 1), gt_poses(gt_index), t);
        gt_R = quat2rotm(gt_orientation);
        gt_T = [gt_R, gt_position'; 0 0 0 1];
        if i > 1
            % inv(inv(prev_gt_T) * gt_T) * (inv(prev_T) * T)
            rpes{i - 1} = (prev_gt_T \ gt_T) \ (prev_T \ T);
        end
        prev_gt_T = gt_T;
        prev_T = T;
    end
end