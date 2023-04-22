function pointcloud = from_PointCloud2(msg)
    pointcloud = struct;
    field_names = rosReadAllFieldNames(msg);
    for field_name = field_names
        pointcloud.(field_name{1}) = rosReadField(msg, field_name{1});
    end
end