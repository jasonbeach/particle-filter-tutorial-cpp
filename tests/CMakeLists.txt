
add_library(catch_main 
tests/tests-main.cpp)

target_link_libraries(catch_main
PUBLIC
  Catch2::Catch2
)