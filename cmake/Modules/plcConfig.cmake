INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_PLC plc)

FIND_PATH(
    PLC_INCLUDE_DIRS
    NAMES plc/api.h
    HINTS $ENV{PLC_DIR}/include
        ${PC_PLC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    PLC_LIBRARIES
    NAMES gnuradio-plc
    HINTS $ENV{PLC_DIR}/lib
        ${PC_PLC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PLC DEFAULT_MSG PLC_LIBRARIES PLC_INCLUDE_DIRS)
MARK_AS_ADVANCED(PLC_LIBRARIES PLC_INCLUDE_DIRS)

