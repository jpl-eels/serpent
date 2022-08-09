# Stereo image pair frame number, starting from 0
int32 frame_number
# Maximum match id that has been assigned
int32 max_match_id
# Match id of the oldest match in the current tracked matches
int32 longest_tracked_match_id
# Number of features tracked in the left and right images
int32[2] tracked_kp_count
# Number of tracked matches
int32 tracked_match_count
# Number of features extracted in the left and right images
int32[2] extracted_kp_count
# Number of extracted features remaining after proximity filtering
int32[2] filtered_extracted_kp_count
# Number of new matches
int32 new_match_count
# Total number of matches (should always equal tracked_match_count + new_match_count)
int32 total_match_count
