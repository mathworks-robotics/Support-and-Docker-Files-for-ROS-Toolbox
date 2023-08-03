#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME msgprototest)

# read all .proto file names expect CoSimMsgs.proto 
file(GLOB PROTO_FILES
    "${PROJECT_SOURCE_DIR}/msgsproto/pkgtest/*.proto"
)

# generate proto source and header for each proto
foreach(ARG ${PROTO_FILES})
    message(" Building ... ${ARG}") 
    execute_process (
    COMMAND bash -c "protoc --proto_path=${PROJECT_SOURCE_DIR}/msgsproto/pkgtest/ --cpp_out=${PROJECT_SOURCE_DIR}/msgsproto/  ${ARG}")
endforeach()


