idf_component_register(
        SRCS "./src/sfud.c" "./src/sfud_sfdp.c" "sfud_port.c"
        INCLUDE_DIRS "." "./inc"
	REQUIRES driver spi_flash esp_timer 
)
