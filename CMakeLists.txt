cmake_minimum_required(VERSION 3.30)
project(parafoil)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

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

add_executable(parafoil)

target_sources(parafoil
        PRIVATE
            main.cpp dynamics.cpp ode45.cpp dynamicprogramming.cpp
        PUBLIC
            FILE_SET CXX_MODULES
            FILES parafoil.ixx)


