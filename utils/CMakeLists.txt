add_library(miao.utils SHARED
        sampler.cc
        ${CMAKE_SOURCE_DIR}/deps/utils/timer.cc
)

install(TARGETS miao.utils
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)