option(CAUV_BUILD_SIM "True if the simulator should be configured and built" OFF)

if (${CAUV_BUILD_SIM})

    project (sim)

    #Open Scene Graph
    find_package(OSG REQUIRED)

    # Open CV
    find_package (OpenCV REQUIRED)

    add_executable(
        sim
        sim.cpp
        sim_camera.cpp
        sim_sonar.cpp
        objects/barracuda.cpp
        objects/buoy.cpp
        objects/water.cpp
        objects/nodetrail.cpp
        FixedNodeTrackerManipulator.cpp
    )
    depends_on_messages(sim)
    cauv_install(sim)

    target_link_libraries (
        sim
        common
        cauv_debug
        opencv_image
        ${OSG_LIBRARY}
        ${OSGVIEWER_LIBRARY}
        ${OSGGA_LIBRARY}
        ${OSGDB_LIBRARY}
        ${OPENTHREADS_LIBRARY}
    )
endif()
