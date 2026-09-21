# Fixed logical source ID. Zero is invalid.
uint16 source_id

# Incremented for each new source sample/result; wraps naturally.
uint32 seq

# Source/result generation time as Unix microseconds; zero means unavailable.
uint64 stamp_us
