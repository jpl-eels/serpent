function rigid = pose_to_rigid3d(pose)
    t = extract_position(pose);
    q = extract_quaternion(pose);
    if abs(norm(q) - 1) > 1.0e-12
        error(join(["Quaternion was not normalised ( norm = ", ...
            num2str(norm(q)), "): ", mat2str(q)]));
    end
    R = quat2rotm(q);
    rigid = rigid3d(R, t);
end