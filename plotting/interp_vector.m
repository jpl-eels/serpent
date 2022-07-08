function vec_interp = interp_vector(t1, t2, vec1, vec2, t)
    interp_coeff = (t - t1) / (t2 - t1);
    if (interp_coeff < 0 || interp_coeff > 1)
        error("Interpolation coefficient outside of [0, 1] bounds");
    end
    vec_interp = vec1 + interp_coeff * (vec2 - vec1);
end