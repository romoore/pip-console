SET(SourceFiles
  pip_console.cpp
  cons_ncurses.cpp
)


add_executable (pip_console ${SourceFiles})
target_link_libraries (pip_console owl-common owl-sensor pthread usb-1.0 ncurses panel)

INSTALL(TARGETS pip_console RUNTIME DESTINATION bin/owl)
