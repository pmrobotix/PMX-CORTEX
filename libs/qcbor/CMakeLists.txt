cmake_minimum_required(VERSION 3.10)
project(qcbor LANGUAGES C)

add_library(qcbor STATIC
    src/qcbor_encode.c
    src/qcbor_decode.c
    src/UsefulBuf.c
    src/ieee754.c
    src/qcbor_err_to_str.c
)

target_include_directories(qcbor PUBLIC inc)
