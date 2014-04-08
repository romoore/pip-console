SET(SourceFiles
  pip_sense_layer.cpp
)

add_executable (pip_sense_layer ${SourceFiles})
target_link_libraries (pip_sense_layer owl-common owl-sensor pthread bcm2835)

INSTALL(TARGETS pip_sense_layer RUNTIME DESTINATION bin/owl)
