# One Annex-B H.265 data chunk. Chunks with the same frame_sequence belong to one frame.
std_msgs/Header header
string codec
string source_encoding
uint32 width
uint32 height
uint64 frame_sequence
bool key_frame
uint8[] data
