set(CPP_SOURCE aux.cpp)
set(HEADERS aux.h)

find_package(OpenCV REQUIRED)

add_library(Auxiliar  ${CPP_SOURCE})

target_include_directories(
    Auxiliar PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>"
)

install(TARGETS Auxiliar DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include)