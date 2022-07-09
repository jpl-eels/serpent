% Compute the Absolute Pose Error and Relative Pose Error, using
% interpolation to estimate ground truth at the query timestamps.
% 
% We can't use interp1 or interp_vector because poses interpolate
% differently
function [apes, ape_timestamps, rpes, rpe_timestamps] = ...
        compute_pose_errors(gt_timestamps, gt_poses, timestamps, poses)
    % Filter within gt bounds
    timestamps_within_gt = timestamps <= gt_timestamps(end) & ...
        timestamps >= gt_timestamps(1);
    ape_timestamps = timestamps(timestamps_within_gt);
    poses = poses(timestamps_within_gt);
    rpe_timestamps = ape_timestamps(2:end);

    % Setup
    ape_size = length(ape_timestamps);
    apes = cell(ape_size, 1);
    rpes = cell(ape_size - 1, 1);
    gt_index = 1;
    prev_gt_T = eye(4);
    prev_T = eye(4);

    % Computation
    for i = 1:ape_size
        t = ape_timestamps(i);
        pose = poses(i);
        T = pose_to_matrix(pose);
        while gt_timestamps(gt_index) < t
            gt_index = gt_index + 1;
        end
        if gt_timestamps(gt_index) == t
            gt_T = pose_to_matrix(gt_poses(gt_index));
        else
            [gt_position, gt_orientation] = interp_transform(...
                gt_timestamps(gt_index - 1), gt_timestamps(gt_index), ...
                gt_poses(gt_index - 1), gt_poses(gt_index), t);
            gt_R = quat2rotm(gt_orientation);
            gt_T = [gt_R, gt_position'; 0 0 0 1];
        end
        apes{i} = gt_T \ T; % inv(gt_T) * pose_T
        if i > 1
            % Equivalent to inv(inv(prev_gt_T) * gt_T) * (inv(prev_T) * T)
            rpes{i - 1} = (prev_gt_T \ gt_T) \ (prev_T \ T);
        end
        prev_gt_T = gt_T;
        prev_T = T;
    end
end