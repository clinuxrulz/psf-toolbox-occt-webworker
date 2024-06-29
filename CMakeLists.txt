cmake_minimum_required (VERSION 3.8)

project(PsfToolboxOcctWebWorker CXX)
set(CMAKE_CXX_STANDARD 14)

add_link_options(--bind -sALLOW_MEMORY_GROWTH=1 -sEXPORTED_RUNTIME_METHODS=['FS'] -O0)

include_directories(
    "C:/nlohmann-json/single_include"
    "C:/OCCT/work/occt-wasm32-make/inc"
)
link_directories("C:/OCCT/work/occt-wasm32-make/lin32/clang/lib")
link_libraries(
    TKernel
    TKBool
    TKBRep
    TKBO
    TKFillet
    TKG2d
    TKG3d
    TKGeomAlgo
    TKGeomBase
    TKMath
    TKOffset
    TKPrim
    TKShHealing
    TKTopAlgo
)

add_executable(PsfToolboxOcctWebWorker main.cpp)
