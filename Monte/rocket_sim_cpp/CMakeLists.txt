cmake_minimum_required(VERSION 3.16)
project(RocketSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Compiler warnings ----
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Core simulation library ----
add_library(rocket_sim STATIC
    sensors.cpp
    controller.cpp
    motor.cpp
    forces.cpp
    rocket.cpp
    magnetic_model.cpp
)

target_include_directories(rocket_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ---- Simulation executable ----
add_executable(sim_main main.cpp)
target_link_libraries(sim_main PRIVATE rocket_sim)

# ---- Header-only files (listed for IDE visibility) ----
# atmosphere.h, isa_atmosphere.h, magnetic_model.h,
# rocket_state.h, sensor_config.h, properties.h, vectors.h
