# compiler option for endianness of the platform
#add_definitions(-DSBG_PLATFORM_BIG_ENDIAN)
add_definitions(-DSBG_PLATFORM_LITTLE_ENDIAN)

if(WIN32)
    file(GLOB COM_SRCS
        src/comWrapper/comSerialWin.c
        src/comWrapper/comDataLog.c  )
else()
    file(GLOB COM_SRCS
        src/comWrapper/comSerialUnix.c
        src/comWrapper/comDataLog.c  )
endif(WIN32)

file(GLOB COM_HEADER src/comWrapper/[^.]*.h  )

file(GLOB PROTOCOL_SRCS src/protocol/[^.]*.c)
file(GLOB PROTOCOL_HEADERS src/protocol/[^.]*.h)

file(GLOB PROTOCOL_ext_SRCS src/protocol/extDevices/[^.]*.c)
file(GLOB PROTOCOL_ext_HEADERS src/protocol/extDevices/[^.]*.h)

file(GLOB TIME src/time/sbgTime.c src/time/sbgTime.h)
file(GLOB SBG_BASE src/sbgCom.c src/sbgCom.h src/sbgCommon.h src/sbgComVersion.h)

add_library(sbgCom STATIC
            ${COM_SRCS}
            ${COM_HEADER}
            ${PROTOCOL_SRCS}
            ${PROTOCOL_HEADERS}
            ${PROTOCOL_ext_SRCS}
            ${PROTOCOL_ext_HEADERS}
            ${TIME}
            ${SBG_BASE}
            )
