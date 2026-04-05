px4_add_module(
        MODULE examples__stability
        MAIN stability
        STACK_MAIN 10000
        SRCS
                stability.cpp
        DEPENDS
                px4_platform_common
)
