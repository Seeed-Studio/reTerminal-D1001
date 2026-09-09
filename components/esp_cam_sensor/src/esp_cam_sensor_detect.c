#include "esp_cam_sensor_detect.h"

/**
 * @brief Get the array of camera sensor detect functions.
 *
 * This fork always uses the linker-section (dynamic link) detect method,
 * so the array boundaries come from the SURROUND() symbols in linker.lf.
 *
 * @param array_start_ptr Pointer to the start of the array.
 * @param array_end_ptr Pointer to the end of the array.
 */
void esp_cam_sensor_detect_get_array(esp_cam_sensor_detect_fn_t **array_start_ptr, esp_cam_sensor_detect_fn_t **array_end_ptr)
{
    *array_start_ptr = (esp_cam_sensor_detect_fn_t *)&__esp_cam_sensor_detect_fn_array_start;
    *array_end_ptr = (esp_cam_sensor_detect_fn_t *)&__esp_cam_sensor_detect_fn_array_end;
}
