# Get the compiler specific includes and store them in GXX_INCLUDES in the parent scope
execute_process(COMMAND echo COMMAND ${CMAKE_CXX_COMPILER} -Wp,-v -x c++ - -fsyntax-only ERROR_VARIABLE GXX_OUTPUT)
set(ENV{GXX_OUTPUT} ${GXX_OUTPUT})
execute_process(COMMAND echo ${GXX_OUTPUT} COMMAND grep "^\ " COMMAND sed "s#\ ##g" COMMAND tr "\n" "\\;" OUTPUT_VARIABLE GXX_INCLUDES)

message (STATUS "Set GXX_INCLUDES to ... (To many to list)") # ${GXX_INCLUDES}