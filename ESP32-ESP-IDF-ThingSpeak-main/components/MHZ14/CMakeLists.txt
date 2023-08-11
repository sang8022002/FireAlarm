set(app_src mhz14a.c)
set(pre_req vfs fatfs)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
