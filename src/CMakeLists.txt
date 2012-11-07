SET(SourceFiles
  pip_sense_layer.cpp
)

add_executable (pip_sense_layer ${SourceFiles})
target_link_libraries (pip_sense_layer owl owl-sensor pthread usb)

INSTALL(TARGETS pip_sense_layer RUNTIME DESTINATION bin/owl)
