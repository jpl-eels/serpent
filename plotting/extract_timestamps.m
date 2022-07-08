function timestamps = extract_timestamps(headered_msgs)
    sec = cellfun(@(m) double(m.Header.Stamp.Sec), headered_msgs);
    nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), headered_msgs);
    timestamps = sec + nsec / 1.0e9;
end