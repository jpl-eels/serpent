function create_save_directory(save_directory)
    [mkdir_status, msg, msgID] = mkdir(save_directory);
    if mkdir_status
        fprintf("Save directory: %s\n", save_directory);
    else
        fprintf('Failed to create directory [msgID: %s, msg: %s\n', ...
            msgID, msg);
    end
end