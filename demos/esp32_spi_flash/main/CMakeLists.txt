set (SOURCES)

list (APPEND SOURCES "main.c")


idf_component_register(SRCS "${SOURCES}"
                    REQUIRES "spi_flash" "FlashDB")
