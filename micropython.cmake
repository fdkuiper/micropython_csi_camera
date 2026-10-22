# ------------------------------------------------------------------------------
#  User-mod camera component
# ------------------------------------------------------------------------------
add_library(usermod_camera INTERFACE)

# -----------------------------------------------------------------------------
#  Include directories
# -----------------------------------------------------------------------------
set(CAM_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}
)

# -----------------------------------------------------------------------------
#  Source files
# -----------------------------------------------------------------------------
set(CAM_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/csi.c
)

# -----------------------------------------------------------------------------
#  ESP-IDF component properties
# -----------------------------------------------------------------------------
idf_component_get_property(COMP_UNITY_INC    unity          INCLUDE_DIRS)
idf_component_get_property(COMP_UNITY_DIR    unity          COMPONENT_DIR)

idf_component_get_property(COMP_CAM_INC      esp_driver_cam INCLUDE_DIRS)
idf_component_get_property(COMP_CAM_DIR      esp_driver_cam COMPONENT_DIR)

idf_component_get_property(COMP_SENSOR_INC   sensor_init    INCLUDE_DIRS)
idf_component_get_property(COMP_SENSOR_DIR   sensor_init    COMPONENT_DIR)

idf_component_get_property(COMP_ISP_INC      esp_driver_isp INCLUDE_DIRS)
idf_component_get_property(COMP_ISP_DIR      esp_driver_isp COMPONENT_DIR)

idf_component_get_property(COMP_MM_INC       esp_mm         INCLUDE_DIRS)
idf_component_get_property(COMP_MM_DIR       esp_mm         COMPONENT_DIR)

idf_component_get_property(COMP_JPEG_INC       esp_driver_jpeg         INCLUDE_DIRS)
idf_component_get_property(COMP_JPEG_DIR       esp_driver_jpeg         COMPONENT_DIR)
# -----------------------------------------------------------------------------
#  Append full paths to CAM_INCLUDES
# -----------------------------------------------------------------------------

list(TRANSFORM COMP_UNITY_INC   PREPEND ${COMP_UNITY_DIR}/)
list(APPEND   CAM_INCLUDES      ${COMP_UNITY_INC})

list(TRANSFORM COMP_CAM_INC     PREPEND ${COMP_CAM_DIR}/)
list(APPEND   CAM_INCLUDES      ${COMP_CAM_INC})

list(TRANSFORM COMP_SENSOR_INC  PREPEND ${COMP_SENSOR_DIR}/)
list(APPEND   CAM_INCLUDES      ${COMP_SENSOR_INC})

list(TRANSFORM COMP_ISP_INC     PREPEND ${COMP_ISP_DIR}/)
list(APPEND   CAM_INCLUDES      ${COMP_ISP_INC})

list(TRANSFORM COMP_MM_INC      PREPEND ${COMP_MM_DIR}/)
list(APPEND   CAM_INCLUDES      ${COMP_MM_INC})

list(TRANSFORM COMP_JPEG_INC      PREPEND ${COMP_JPEG_DIR}/)
list(APPEND   CAM_INCLUDES      ${COMP_JPEG_INC})
# -----------------------------------------------------------------------------
#  Attach sources & includes
# -----------------------------------------------------------------------------
target_sources(usermod_camera INTERFACE ${CAM_SOURCES})
target_include_directories(usermod_camera INTERFACE ${CAM_INCLUDES})

# -----------------------------------------------------------------------------
#  Link usermod interface
# -----------------------------------------------------------------------------
target_link_libraries(usermod INTERFACE usermod_camera)

# The board fork opts into the OV9281 backport; no managed components are edited.
if(MICROPY_CAMERA_OV9281)
    target_sources(usermod_camera INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/sensors/ov9281/ov9281.c)
    target_include_directories(usermod_camera INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/sensors/ov9281/include
        ${CMAKE_CURRENT_LIST_DIR}/sensors/ov9281/private_include)
    target_compile_definitions(usermod_camera INTERFACE MICROPY_CAMERA_OV9281=1)
endif()
