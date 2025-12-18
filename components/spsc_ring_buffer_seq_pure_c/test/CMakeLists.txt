# Unit tests for SPSC Ring Buffer (Pure C)

idf_component_register(
    SRCS "test_spsc_ring_buffer_seq_c.c"
    INCLUDE_DIRS "."
    REQUIRES "spsc_ring_buffer_seq_pure_c" "unity"
)

