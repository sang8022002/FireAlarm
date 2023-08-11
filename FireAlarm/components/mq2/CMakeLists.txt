set(app_src mq2.c)
set(pre_req vfs fatfs)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
# set(app_src co.c)
# set(pre_req driver log)
# idf_component_register(SRCS ${app_src}
#                     INCLUDE_DIRS "."
#                     REQUIRES ${pre_req})
# idf_component_register(   SRCS "mq2.c" "co.h"
#                     INCLUDE_DIRS "."
#                     REQUIRES "esp_adc_cal" )

# idf_component_register(SRCS "co.h"
#                         INCLUDE_DIRS "."
#                         REQUIRES fatfs esp_adc_cal)
# idf_component_register(SRCS
#                         "co.h"
#                         INCLUDE_DIRS .
#                         REQUIRES fatfs esp_adc_cal
#                         )
