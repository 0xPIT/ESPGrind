SET(SOURCES 
    touch-CST328.c
    ST7789T.c
    PCF85063.c
    waveshare-esp32s3-touch2.8.bsp.c)
    
add_library(bsp ${SOURCES})

