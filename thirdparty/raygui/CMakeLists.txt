add_library(raygui
  raygui.c
)


target_link_libraries(raygui PUBLIC raylib)
target_include_directories(raygui PUBLIC .)
