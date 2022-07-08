function [ape_timestamps, ape] = compute_ape(gt_timestamps, gt_poses, ...
        timestamps, poses)
    ape_timestamps = timestamps(timestamps <= gt_timestamps(end));
    size = length(ape_timestamps);
    ape = cell(size, 1);
    % We can't use interp1 because we need to interpolate poses
    gt_index = 1;
    for i = 1:size
        t = ape_timestamps(i);
        pose = poses(i);
        pose_T = pose_to_matrix(pose);
        while gt_timestamps(gt_index) < t
            gt_index = gt_index + 1;
        end
        [gt_position, gt_orientation] = interp_transform(...
            gt_timestamps(gt_index - 1), gt_timestamps(gt_index), ...
            gt_poses(gt_index - 1), gt_poses(gt_index), t);
        gt_R = quat2rotm(gt_orientation);
        gt_T = [gt_R, gt_position'; 0 0 0 1];
        % inv(gt_T) * pose_T
        ape{i} = gt_T \ pose_T;
    end
end