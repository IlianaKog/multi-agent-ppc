cmake_minimum_required(VERSION 3.15)
project(PPC_MultiAgent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find Boost (specifically Odeint, which is header-only, but we might need system basics)
find_package(Boost REQUIRED)

add_executable(PPC_Simulation main.cpp MasSystem.cpp)

target_link_libraries(PPC_Simulation PRIVATE Eigen3::Eigen)

target_include_directories(PPC_Simulation PRIVATE 
    ${Boost_INCLUDE_DIRS}
    "C:/msys64/ucrt64/include/eigen3"
)

# Optional: Enable optimization for faster simulation
target_compile_options(PPC_Simulation PRIVATE "-O3")
