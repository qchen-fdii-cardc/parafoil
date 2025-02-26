cmake_minimum_required(VERSION 3.20)
project(parafoil)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create directories for source and include files
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the directories if they don't exist
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR})
file(MAKE_DIRECTORY ${PROJECT_INCLUDE_DIR})

# using multiple cores for compilation
######################使用多处理器编译########################
IF (WIN32)
		if(MSVC)
				OPTION(USE_MP "use multiple" ON)
				OPTION(ProjectConfig_Global_COMPILE_FLAGS_WITH_MP 
					"Set The Global Option COMPILE_FLAGS /MP to target." ON)
				if(ProjectConfig_Global_COMPILE_FLAGS_WITH_MP OR USE_MP)
					set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
					set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
				endif()
				set(VS_STARTUP_PROJECT ${PROJECT_NAME})
		ENDIF(MSVC)
ENDIF()

# Add include directories
include_directories(${PROJECT_INCLUDE_DIR})

# Define source files
set(COMMON_SOURCES
    ${PROJECT_SOURCE_DIR}/dynamics.cpp
    ${PROJECT_SOURCE_DIR}/ode45.cpp
    ${PROJECT_SOURCE_DIR}/dynamicprogramming.cpp
    ${PROJECT_SOURCE_DIR}/parafoil.cpp
)

# Define header files
set(COMMON_HEADERS
    ${PROJECT_INCLUDE_DIR}/parafoil.hpp
)

# Parafoil executable
add_executable(parafoil)
target_sources(parafoil
        PRIVATE
            ${PROJECT_SOURCE_DIR}/main.cpp
            ${COMMON_SOURCES}
)

# DE demo executable
add_executable(de_demo)
target_sources(de_demo
        PRIVATE
            ${PROJECT_SOURCE_DIR}/de.cpp
            ${COMMON_SOURCES}
)

# NSGA2 demo executable (commented out)
# add_executable(nsga2_demo)
# target_sources(nsga2_demo
# 		PRIVATE
# 			${PROJECT_SOURCE_DIR}/nsgii.cpp
# 			${COMMON_SOURCES}
# )