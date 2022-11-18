function frame_ids = extract_frame_ids(headered_msgs)
    frame_ids = cellfun(@(m) string(m.Header.FrameId), headered_msgs);
end