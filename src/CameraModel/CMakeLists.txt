add_library(CameraModel
    Camera_Intrinsics.cpp
)

add_executable(CameraModelTestNode
    CameraModelTestNode.cpp
)

target_link_libraries(CameraModelTestNode
    CameraModel
    gtest
)

add_test(NAME CameraModelTestNode COMMAND CameraModelTestNode)