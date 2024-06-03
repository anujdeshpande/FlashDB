idf_component_register(SRCS "porting/fal_flash_esp32_port.c" "./flashdb/samples/tsdb_sample.c" "./flashdb/samples/kvdb_basic_sample.c" "./flashdb/samples/kvdb_type_blob_sample.c" "./flashdb/samples/kvdb_type_string_sample.c" "./flashdb/src/fdb_tsdb.c" "./flashdb/src/fdb_file.c" "./flashdb/src/fdb_utils.c" "./flashdb/src/fdb_kvdb.c" "./flashdb/src/fdb.c" "./fal/src/fal_flash.c" "./fal/src/fal_partition.c" "./fal/src/fal.c" "./fal/src/fal_rtt.c"  
                    INCLUDE_DIRS "inc" "./fal/inc" "./flashdb/inc"
                    REQUIRES "spi_flash" "esp_partition" "esp_rom" "driver")
