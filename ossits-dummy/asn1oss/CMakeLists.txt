#
# This is the CMakeLists.txt to bind the controller lib of the oss generator
#
# Authors:
# - Christoph Pilz (christoph.pilz@v2c2.at)
# 

cmake_minimum_required(VERSION 3.9)

message(STATUS "--> Configure build of |-ossits-asn1oss")

project(ossits-asn1oss VERSION 1.0 DESCRIPTION "asn1oss part of ossits")

add_library(ossits-asn1oss SHARED IMPORTED)

# you may switch between time optimized and space optimized library
# - you have to regenerate the library!
set_target_properties(ossits-asn1oss PROPERTIES VERSION ${PROJECT_VERSION}
        LINKER_LANGUAGE CXX
        COMPILE_FLAGS "-losstoed"
#        COMPILE_FLAGS "-lasn1code -liaapi"
        LINK_FLAGS "-losstoed")
#        LINK_FLAGS "-lasn1code -liaapi")

# find libraries
# osstoed can not be used together iwth asn1code -> ossinit() retcode = 7
# - osstoed: time optimized lib
# - asn1code: space optimized lib
# - libiaapi: can be linked with asn1code for additional features
#find_library(ASN1CODE
#        NAMES
#        "asn1code"
#        HINTS
#        ${CMAKE_CURRENT_LIST_DIR}/lib
#        )
#find_library(IAAPI
#        NAMES
#        "iaapi"
#        HINTS
#        ${CMAKE_CURRENT_LIST_DIR}/lib
#        )
find_library(OSSTOED
        NAMES
        "osstoed"
        HINTS
        ${CMAKE_CURRENT_LIST_DIR}/lib
        )

#message(STATUS "--> asn1code path: " ${ASN1CODE})
#message(STATUS "--> iaapi path: " ${IAAPI})
#message(STATUS "--> osstoed path: " ${OSSTOED})

# link the target against the following libraries
target_link_libraries(ossits-asn1oss
        INTERFACE
#        ${ASN1CODE}
#        ${IAAPI}
        ${OSSTOED}
        )

message(STATUS "--> Configure build of |-ossits-asn1oss - done")
