cmake_minimum_required (VERSION 3.8)

project(PsfToolboxOcctWebWorker CXX)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(
    -fexceptions
    -gseparate-dwarf
    --use-port=zlib
)

# -O3 for release
add_link_options(
    -fexceptions
    -O0
    --bind
    -sEXPORTED_RUNTIME_METHODS=['FS']
    -sINITIAL_MEMORY=100MB
    -sMAXIMUM_MEMORY=4GB
    -sSTACK_SIZE=5MB
    -sALLOW_MEMORY_GROWTH=1
    --use-port=zlib
)

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
    TKHLR
    TKMath
    TKMesh
    TKOffset
    TKPrim
    TKShHealing
    TKTopAlgo
)

add_executable(PsfToolboxOcctWebWorker main.cpp gzip.cpp base64.cpp util.cpp)
