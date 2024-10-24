# Copyright 2024 NXP
# SPDX-License-Identifier: BSD-3-Clause

if(CONFIG_MCUX_COMPONENT_middleware.wireless.genfsk)

    # add other CmakeLists files if you need to
    # mcux_add_cmakelists(${PATH})

    # add source files and header file - header file are required for IAR project generation
    mcux_add_source(
        SOURCES 
            source/genfsk_isr.c
            source/genfsk_ll.c
            source/genfsk_utils.c
            source/genfsk_time.c
            interface/genfsk_interface.h
            interface/genfsk_sw_version.h
            interface/genfsk_ll.h
            interface/genfsk_utils.h
            interface/dtest_ctrl.h     
    )

    # add include path
    mcux_add_include(
        INCLUDES 
            interface
    )
endif()