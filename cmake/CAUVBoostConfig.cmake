
find_package (Boost 1.40)

if (Boost_VERSION LESS 104000)
    if (EXISTS /societies/cauv/install/)
        message(WARNING "Boost version too low (${Boost_VERSION}), assuming SRCF paths")
        set (Boost_INCLUDE_DIRS "/societies/cauv/install/include/")
        set (Boost_LIBRARY_DIRS "/societies/cauv/install/lib/")
    else (EXISTS /societies/cauv/install/)
        message(WARNING "Boost version too low (${Boost_VERSION}), assuming user paths")
        set (Boost_INCLUDE_DIRS "~/install/include/")
        set (Boost_LIBRARY_DIRS "~/install/lib/" "~/install/lib64/")
    endif (EXISTS /societies/cauv/install/)
endif (Boost_VERSION LESS 104000)

