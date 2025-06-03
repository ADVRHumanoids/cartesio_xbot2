cmake_minimum_required(VERSION 3.10)

add_executable(test_deserialize test_deserialize.cpp)

install(TARGETS test_deserialize
        DESTINATION bin)

install(FILES serialized.txt 
        DESTINATION bin) 