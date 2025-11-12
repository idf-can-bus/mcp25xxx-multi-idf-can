idf_component_register(
    SRCS 
        "src/mcp2515_multi.c"
        "src/mcp2515_multi_internal.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)

