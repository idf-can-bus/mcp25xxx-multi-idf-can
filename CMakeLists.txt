idf_component_register(
    SRCS 
        "src/mcp25xxx_multi.c"
        "src/mcp25xxx_multi_internal.c"
        "components/spsc_ring_buffer_seq_pure_c/spsc_ring_buffer_seq_c.c"
    INCLUDE_DIRS 
        "include"
    PRIV_INCLUDE_DIRS 
        "src"  # Internal headers (not exposed to other components)
        "components/spsc_ring_buffer_seq_pure_c/include"
    REQUIRES driver esp_timer
)

