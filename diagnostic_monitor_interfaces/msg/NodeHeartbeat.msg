# Fixed logical node ID. Zero is invalid.
uint16 node_id

# Non-zero per-process boot ID. Constant for the process lifetime.
uint64 boot_id

# Incremented for each heartbeat publication; wraps naturally.
uint32 heartbeat_seq

# Incremented only when the selected real business work completes.
uint32 work_seq

# Producer-specific state in [0, 4]; see the publishing node's definition.
uint8 state
