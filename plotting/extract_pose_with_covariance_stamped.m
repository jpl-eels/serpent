function data = extract_pose_with_covariance_stamped(raw_data)
    data.frame_ids = extract_frame_ids(raw_data);
    data.timestamps = extract_timestamps(raw_data);
    poses_with_covs = cellfun(@(m) struct(m.Pose), raw_data);
    poses = arrayfun(@(m) struct(m.Pose), poses_with_covs);
    data.positions = extract_positions(poses);
    data.rots_quat = extract_quaternions(poses);
    rots_axang = quat2axang(data.rots_quat);
    data.rots_axang = rots_axang(:, 1:3) .* rots_axang(:, 4);
    covariance = arrayfun(@(m) m.Covariance, poses_with_covs, ...
        'UniformOutput', false);
    data.covariance = cell(length(covariance), 1);
    data.diagonals = zeros(length(covariance), 6);
    for i = 1:length(covariance)
        data.covariance{i} = reshape(covariance{i}, 6, 6);
        data.diagonals(i, :) = diag(data.covariance{i});
    end
    data.sigmas = sqrt(data.diagonals);
%     data.covariance = cellfun(@(m) reshape(covariance, 6, 6), ...
%         covariance);
end