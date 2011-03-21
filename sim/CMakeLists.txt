option(CAUV_BUILD_SIM "True if the simulator should be configured and built" OFF)

if (${CAUV_BUILD_SIM})

    project (sim)


    option(CAUV_BUILD_SIM "True if the simulator should be configured and built" OFF)


    #Open Scene Graph
    find_package(OSG REQUIRED)
    FIND_OSG_LIBRARY(OSGOCEAN_LIBRARY osgOcean)

    # boost:
    find_package (Boost 1.40 REQUIRED)
    set (LIBS ${LIBS} ${Boost_LIBRARIES})
    include_directories (SYSTEM ${Boost_INCLUDE_DIRS})
    link_directories (${Boost_LIBRARY_DIRS})

    # Open CV
    find_package (OpenCV REQUIRED)

    # for <common/includesstuff> etc
    include_directories (${AUV_SOURCE_DIR})

    # sources
    add_executable (
        sim EXCLUDE_FROM_ALL
        sim-main.cpp
        simulator.cpp
        worldmodel.cpp
        validators.cpp
        visuals/skydome.cpp
        visuals/spheresegment.cpp
        visuals/oceanscene.cpp
        visuals/pipscreen.cpp
        sensors/camera.cpp
        auvs/simulatedauv.cpp
        auvs/redherring.cpp
    )

    # libs
    target_link_libraries (
        sim
        common
        cauv_debug
        model
        ${OpenCV_LIBS}
        ${OSG_LIBRARY}
        ${OSGVIEWER_LIBRARY}
        ${OSGOCEAN_LIBRARY}
    )

    add_custom_command(
        TARGET sim

        POST_BUILD
        COMMAND cp -R "${sim_SOURCE_DIR}/resources" "${sim_BINARY_DIR}/"
        COMMENT "Copying resources for sim"
    )

endif()
