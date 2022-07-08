% Computes the absolute and relative error of vectors with respect to
% reference vectors. For each vector in vectors for which the timestamp
% is before the final timestamp of the reference timestamps, there is an AE
% and an RE, except for the first vector for which there is no RE. Thus the
% RE matrix will be one row shorter than the AE and starts at the second
% timestamp.
function [ae, re] = compute_vector_errors(reference_timestamps, ...
        reference, timestamps, vectors)
    timestamps = timestamps(timestamps <= reference_timestamps(end));
    valid_size = length(timestamps);
    vectors_size = size(vectors);
    ae = zeros(valid_size, vectors_size(2));
    re = zeros(valid_size - 1, vectors_size(2));
    ref_index = 1;
    prev_reference_vector = zeros(1, vectors_size(2));
    prev_vector = zeros(1, vectors_size(2));
    for i = 1:valid_size
        t = timestamps(i);
        vector = vectors(i, :);
        while reference_timestamps(ref_index) < t
            ref_index = ref_index + 1;
        end
        reference_vector = interp_vector( ...
            reference_timestamps(ref_index - 1), ...
            reference_timestamps(ref_index), ...
            reference(ref_index - 1, :), reference(ref_index, :), t);
        ae(i, :) = vector - reference_vector;
        if i > 1
            re(i - 1, :) = (reference_vector - prev_reference_vector) - ...
                (vector - prev_vector);
        end
        prev_reference_vector = reference_vector;
        prev_vector = vector;
    end
end