# Try to find ns3 library
# will define:
#  NS3_FOUND
#  NS3_INCLUDE_DIR
#  NS3_LIBRARIES
#

set(ns3_modules

    ns3-aodv
    ns3-applications
    ns3-bridge
    ns3-config-store
    ns3-core
    ns3-csma-layout
    ns3-csma
    ns3-dsdv
    ns3-emu
    ns3-energy
    ns3-flow-monitor
    ns3-internet
    ns3-lte
    ns3-mesh
    ns3-mobility
    ns3-mpi
    ns3-netanim
    ns3-network
    ns3-nix-vector-routing
    ns3-olsr
    ns3-point-to-point-layout
    ns3-point-to-point
    ns3-propagation
    ns3-spectrum
    ns3-stats
    ns3-tap-bridge
    ns3-test
    ns3-tools
    ns3-uan
    ns3-virtual-net-device
    ns3-visualizer
    ns3-wifi
    ns3-wimax
)

foreach(ns3_module ${ns3_modules})
    set(ns3_lib_names ${ns3_lib_names} lib${ns3_module})
endforeach()

find_package(PkgConfig)
pkg_check_modules(PC_NS3 QUIET ${ns3_lib_names})

find_path(NS3_INCLUDE_DIR NAMES ns3/core-module.h
          HINTS ${PC_NS3_INCLUDE_DIRS})

foreach(ns3_lib ${ns3_modules})
    find_library(${ns3_lib}_loc NAMES "${ns3_lib}"
                HINTS ${PC_NS3_LIBRARY_DIRS})
    mark_as_advanced(${ns3_lib}_loc)
    set(NS3_LIBRARIES ${NS3_LIBRARIES} ${${ns3_lib}_loc})
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ns3 DEFAULT_MSG
    NS3_LIBRARIES NS3_INCLUDE_DIR)

mark_as_advanced(NS3_LIBRARIES NS3_INCLUDE_DIR)
