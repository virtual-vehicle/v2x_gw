#
# Root CMakelists.txt of the ossits library
#
# Authors:
# - Christoph Pilz (christoph.pilz@v2c2.at)
# 

cmake_minimum_required(VERSION 3.9)

message(STATUS "--> Configure build of ossitslib")

project(ossits VERSION 1.0 DESCRIPTION "oss.com ITS library")

add_subdirectory(asn1oss)

add_library(ossits SHARED
        src/ossits.cpp
)

set_target_properties(ossits PROPERTIES VERSION ${PROJECT_VERSION})

target_include_directories(ossits PRIVATE ./include/ossits)
target_include_directories(ossits PRIVATE ./asn1oss/include/asn1oss)

# find libraries
# - check asn1oss for usage
#find_library(ASN1CODE
#        NAMES
#        "asn1code"
#        HINTS
#        ${CMAKE_CURRENT_LIST_DIR}/lib
#)
#find_library(IAAPI
#        NAMES
#        "iaapi"
#        HINTS
#        ${CMAKE_CURRENT_LIST_DIR}/lib
#)
find_library(OSSTOED
        NAMES
        "osstoed"
        HINTS
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

# link libraries
target_link_libraries(ossits
        PUBLIC
        #        ${ASN1CODE}
        #        ${IAAPI}
        ${OSSTOED}
)

message(STATUS "--> Configure build of ossits - done")

