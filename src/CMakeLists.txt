file(GLOB_RECURSE SRC_UI ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.c)
file(GLOB_RECURSE SRC_BSP ${CMAKE_CURRENT_SOURCE_DIR}/bsp/cyd-esp32-2432S024C.bsp.c)

idf_component_register(
    SRCS 
        "main.c" 
        "mill.c" 
        "settings.c" 
        ${SRC_UI}
        ${SRC_BSP}
    INCLUDE_DIRS 
        "."
        "ui" 
        "bsp"
)
