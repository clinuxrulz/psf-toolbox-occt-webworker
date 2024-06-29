cmake_minimum_required (VERSION 3.8)

project(PsfToolboxOcctWebWorker CXX)
set(CMAKE_CXX_STANDARD 14)

include_directories("C:/OCCT/work/occt-wasm32-make/inc")
link_directories("C:/OCCT/work/occt-wasm32-make/lin32/clang/lib")

add_executable(PsfToolboxOcctWebWorker main.cpp)
