% Computes the absolute and relative error of vectors with respect to
% reference vectors. For each vector in vectors for which the timestamp
% is before the final timestamp of the reference timestamps, there is an AE
% and an RE, except for the first vector for which there is no RE. Thus the
% RE matrix will be one row shorter than the AE and starts at the second
% timestamp.
function [ae, ae_timestamps, re, re_timestamps] = compute_vector_errors(gt_timestamps, ...
        gt_vectors, timestamps, vectors)
    % Filter within gt bounds
    timestamps_within_gt = timestamps <= gt_timestamps(end) & ...
        timestamps >= gt_timestamps(1);
    ae_timestamps = timestamps(timestamps_within_gt);
    vectors = vectors(timestamps_within_gt, :);
    re_timestamps = ae_timestamps(2:end);

    % Setup
    ae_size = length(ae_timestamps);
    vectors_size = size(vectors);
    ae = zeros(ae_size, vectors_size(2));
    re = zeros(ae_size - 1, vectors_size(2));
    gt_index = 1;
    prev_reference_vector = zeros(1, vectors_size(2));
    prev_vector = zeros(1, vectors_size(2));

    for i = 1:ae_size
        t = ae_timestamps(i);
        vector = vectors(i, :);
        while gt_timestamps(gt_index) < t
            gt_index = gt_index + 1;
        end
        if gt_timestamps(gt_index) == t
            gt_vector = gt_vectors(gt_index, :);
        else
            gt_vector = interp_vector(gt_timestamps(gt_index - 1), ...
                gt_timestamps(gt_index), gt_vectors(gt_index - 1, :), ...
                gt_vectors(gt_index, :), t);
        end
        ae(i, :) = vector - gt_vector;
        if i > 1
            re(i - 1, :) = (gt_vector - prev_reference_vector) - ...
                (vector - prev_vector);
        end
        prev_reference_vector = gt_vector;
        prev_vector = vector;
    end
end