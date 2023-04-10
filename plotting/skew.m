function v_skew = skew(v)
    if ~all(size(v) == [3, 1])
        error("Can only compute skew of 3x1 vectors.");
    end
    v_skew = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end