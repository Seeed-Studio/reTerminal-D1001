idf_component_register(SRCS "esp_lcd_jd9365_8.c" INCLUDE_DIRS "include" PRIV_REQUIRES "esp_driver_gpio" REQUIRES "esp_lcd")

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
