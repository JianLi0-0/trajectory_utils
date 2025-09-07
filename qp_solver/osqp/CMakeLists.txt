# Minimum version required
cmake_minimum_required (VERSION 3.2)

# Project name
project (osqp)

set(OSQP_VERSION "0.0.0" CACHE STRING "The version number of OSQP")
if(NOT OSQP_VERSION STREQUAL "0.0.0")
    configure_file (
        "${PROJECT_SOURCE_DIR}/configure/version.h.in"
        "${PROJECT_SOURCE_DIR}/include/version.h"
    )
endif()

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/out)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/out)



# Detect operating system
# ----------------------------------------------
message(STATUS "We are on a ${CMAKE_SYSTEM_NAME} system")
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(IS_LINUX ON)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    set(IS_MAC ON)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(IS_WINDOWS ON)
endif()



# Set options
# ----------------------------------------------

# Are unittests generated?
option (UNITTESTS "Enable unittests generation" OFF)

# Is the code generated for embedded platforms?
#   1 :   Yes. Matrix update not allowed.
#   2 :   Yes. Matrix update allowed.
if (NOT DEFINED EMBEDDED)
    message(STATUS "Embedded is OFF")
else()
    message(STATUS "Embedded is ${EMBEDDED}")
endif()

# Is printing enabled?
option (PRINTING "Enable solver printing" ON)
if (DEFINED EMBEDDED)
    message(STATUS "Disabling printing for embedded")
    set(PRINTING OFF)
endif()
message(STATUS "Printing is ${PRINTING}")


# Is profiling enabled?
option (PROFILING "Enable solver profiling (timing)" ON)
if (DEFINED EMBEDDED)
    message(STATUS "Disabling profiling for embedded")
    set(PROFILING OFF)
endif()
message(STATUS "Profiling is ${PROFILING}")

# Is user interrupt enabled?
option (CTRLC "Enable user interrupt (Ctrl-C)" ON)
if (DEFINED EMBEDDED)
    message(STATUS "Disabling user interrupt for embedded")
    set(CTRLC OFF)
endif()
message(STATUS "User interrupt is ${CTRLC}")

# Use floats instead of integers
option (DFLOAT "Use float numbers instead of doubles" OFF)
message(STATUS "Floats are ${DFLOAT}")

# Use long integers for indexing
option (DLONG "Use long integers (64bit) for indexing" ON)
if (NOT (CMAKE_SIZEOF_VOID_P EQUAL 8))
	message(STATUS "Disabling long integers (64bit) on 32bit machine")
	set(DLONG OFF)
endif()
message(STATUS "Long integers (64bit) are ${DLONG}")


option (DEBUG "Debug mode" OFF)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set (DEBUG ON)
	message(STATUS "Debug mode is ${DEBUG}")
endif()

# Add code coverage
option (COVERAGE "Perform code coverage" OFF)
message(STATUS "Code coverage is ${COVERAGE}")


# Memory allocators
# ----------------------------------------------

#Report on custom user header options.  This is intended to allow
#users to provide definitions of their own memory functions
# The header should define the functions as follows
#
# define c_malloc mymalloc
# define c_calloc mycalloc
# define c_realloc myrealloc
# define c_free myfree

if(OSQP_CUSTOM_MEMORY)
	message(STATUS "User custom memory management header: ${OSQP_CUSTOM_MEMORY}")
endif()



# Linear solvers dependencies
# ---------------------------------------------
option (ENABLE_MKL_PARDISO "Enable MKL Pardiso solver" ON)
if (DFLOAT)
	message(STATUS "Disabling MKL Pardiso Solver with floats")
	set(ENABLE_MKL_PARDISO OFF)
elseif(DEFINED EMBEDDED)
	message(STATUS "Disabling MKL Pardiso Solver for embedded")
	set(ENABLE_MKL_PARDISO OFF)
endif()
message(STATUS "MKL Pardiso: ${ENABLE_MKL_PARDISO}")


# Generate header file with the global options
# ---------------------------------------------
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure/osqp_configure.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/include/osqp_configure.h
               NEWLINE_STYLE LF)

# Set Compiler flags
# ----------------------------------------------
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # -fPIC


if (NOT MSVC)

    if (COVERAGE)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
	if(FORTRAN)
		set(CMAKE_FORTRAN_FLAGS "${CMAKE_FORTRAN_FLAGS} --coverage")
	endif(FORTRAN)
    endif()

    if (DEBUG)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")
    else()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    endif()

    set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lm")      # Include math
    # Include real time library in linux
    if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
        set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lrt -ldl")
    endif()
endif (NOT MSVC)

# Set sources and includes
# ----------------------------------------------
add_subdirectory (src)
add_subdirectory (include)


# if we are building the Python interface, let's look for Python
# and set some options
# -----------------------------------------------------------------
if (PYTHON)

    # Python include directories need to be passed by the python compilation process
    if (NOT PYTHON_INCLUDE_DIRS)
            message( FATAL_ERROR "You need Python include directories to build the Python interface" )
    endif (NOT PYTHON_INCLUDE_DIRS)

    # Include directories for Python headers
    include_directories(${PYTHON_INCLUDE_DIRS})

    # Pass PYTHON flag to C compiler
    add_definitions(-DPYTHON)

    if (UNITTESTS)
        # Disable unittests
        message(STATUS "Disabling UNITTESTS because we are building Python interface")
        set(UNITTESTS OFF)
    endif (UNITTESTS)

endif (PYTHON)


# if we are building the Matlab interface, let's look for Matlab
# and set some options
# -----------------------------------------------------------------
if (MATLAB)

    find_package(Matlab)

    if (NOT Matlab_FOUND)
        message( FATAL_ERROR "You need Matlab libraries to build the Matlab interface" )
    endif (NOT Matlab_FOUND)

    # Include directories for Matlab headers
    include_directories(${Matlab_INCLUDE_DIRS})

    message(STATUS "Matlab root is " ${Matlab_ROOT_DIR})

    # Pass MATLAB flag to C compiler
    add_definitions(-DMATLAB)

    # Insist on the pre 2018 complex data API
    # so that mxGetPr will work correctly
    add_definitions(-DMATLAB_MEXSRC_RELEASE=R2017b)

    message(STATUS "Using Matlab pre-2018a API for mxGetPr compatibility")

    if (UNITTESTS)
        # Disable unittests
        message(STATUS "Disabling UNITTESTS because we are building Matlab interface")
        set(UNITTESTS OFF)
    endif (UNITTESTS)

endif (MATLAB)

# if we are building the R interface, let's look for R
# and set some options
# -----------------------------------------------------------------
if (R_LANG)

    message(STATUS "We are building the R interface")

    # Look for R libraries
    find_package(R)

    if (NOT R_FOUND)
            message( FATAL_ERROR "You need R libraries to build the R interface" )
    endif (NOT R_FOUND)

    message(STATUS "R exec is: " ${R_EXEC})
    message(STATUS "R root dir is: " ${R_ROOT_DIR})
    message(STATUS "R includes are in: " ${R_INCLUDE_DIRS})

    # Include directories for R headers
    include_directories(${R_INCLUDE_DIRS})

    # Pass R_LANG flag to C compiler
    add_definitions(-DR_LANG)

    if (UNITTESTS)
        # Disable unittests
        message(STATUS "Disabling UNITTESTS because we are building the R interface")
        set(UNITTESTS OFF)
    endif (UNITTESTS)

endif (R_LANG)


# Create Static Library
# ----------------------------------------------

# Add linear system solvers cumulative library
add_subdirectory(lin_sys)

include(GNUInstallDirs)

# Install Headers
# ----------------------------------------------

install(FILES ${osqp_headers} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/osqp")


# If we are building Python/Matlab/R interface:
#   - do not build shared library
#   - do not build demo
if (NOT PYTHON AND NOT MATLAB AND NOT R_LANG AND NOT EMBEDDED)
    # Create osqp shared library
    # NB: Add all the linear system solvers here
    add_library (osqp SHARED ${osqp_src} ${osqp_headers} ${linsys_solvers})

    # Include directories for linear system solvers
    target_include_directories(osqp PRIVATE ${linsys_solvers_includes})

    # Declare include directories for the cmake exported target
    target_include_directories(osqp
                               PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                      "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/osqp>")

    # Install osqp shared library
    install(TARGETS osqp
            EXPORT  OsqpEigen
            LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")

endif (NOT PYTHON AND NOT MATLAB AND NOT R_LANG AND NOT EMBEDDED)
