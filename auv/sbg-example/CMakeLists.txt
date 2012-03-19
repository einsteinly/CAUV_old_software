find_package(SDL)
find_package(GLUT)
find_package(OpenGL)


include_directories("/usr/local/include/sbgCom/")
include_directories(${SDL_INCLUDE_DIR})

# sources
add_executable (
    cube3d

    cube3D.c
    ig500.c
    main.c
    sdlApplication.c
)

target_link_libraries (
    cube3d

    SbgComSerial
    SDL

    ${OPENGL_LIBRARIES}
)
