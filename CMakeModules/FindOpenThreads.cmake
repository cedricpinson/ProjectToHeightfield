# OpenThreads is a C++ based threading library. Its largest userbase 
# seems to OpenSceneGraph so you might notice I accept OSGDIR as an
# environment path.
# I consider this part of the Findosg* suite used to find OpenSceneGraph 
# components.
# Each component is separate and you must opt in to each module.
# 
# Locate OpenThreads
# This module defines
# OPENTHREADS_LIBRARY
# OPENTHREADS_FOUND, if false, do not try to link to OpenThreads
# OPENTHREADS_INCLUDE_DIR, where to find the headers
#
# $OPENTHREADSDIR is an environment variable that would
# correspond to the ./configure --prefix=$OPENTHREADSDIR
# used in building osg.
#
# Created by Eric Wing.

# Header files are presumed to be included like
# #include <OpenThreads/Thread>

# Try the user's environment request before anything else.
FIND_PATH(OPENTHREADS_INCLUDE_DIR OpenThreads/Thread
	$ENV{OPENTHREADSDIR}/include
	${OPENTHREADSDIR}/include
	$ENV{OSGDIR}/include
	${OSGDIR}/include
)

# On OSX, this will prefer the Framework version (if found) over others
# except when the environment is set to a valid location.
# People will have to manually change the cache values of 
# OPENTHREADS_LIBRARY to override this selection.
# OS X is a nasty corner case because CMake's FIND_PATH isn't framework aware.
# Apple provides magic so that #include <OpenThreads/Thread>
# maps to */Library/Frameworks/OpenThreads.framework/Headers/Thread
# but CMake FIND_PATH isn't smart enough to handle this if I specify the 
# file as OpenThreads/Thread since no OpenThreads/Thread
# actually exists.
# But it gets worse. I actually don't want the path mapped to the Headers 
# directory, but only to the directory that holds the framework (e.g 
# /Library/Frameworks). In the past, OS X automatically searched for
# the /Library/Frameworks directories, but with Universal Binaries, the 
# process has changed. Furthermore, it won't cover cases where the frameworks
# are located elsewhere.
IF(NOT OPENTHREADS_INCLUDE_DIR)
	# For usage of #include <Foo/foo.h> with a framework, the FIND_PATH 
	# and traditional -I parameter is useless. The -F parameter must be
	# used. So we will use FIND_PATH to a temporary variable just so 
	# we know if we have a framework and where it is.
	# TODO: Figure out a way $ENV can be used.
	FIND_PATH(OPENTHREADS_INCLUDE_DIR_TMP Thread
		~/Library/Frameworks/OpenThreads.framework/Headers
		/Library/Frameworks/OpenThreads.framework/Headers
	)
	
	# If the Framework style is not found, do the classic approach
	IF(NOT OPENTHREADS_INCLUDE_DIR_TMP)
		FIND_PATH(OPENTHREADS_INCLUDE_DIR OpenThreads/Thread
			/usr/local/include
			/usr/include
			/sw/include # Fink
			/opt/local/include # DarwinPorts
			/opt/csw/include # Blastwave
			/opt/include
		)
	ENDIF(NOT OPENTHREADS_INCLUDE_DIR_TMP)
ENDIF(NOT OPENTHREADS_INCLUDE_DIR)


# I'm not sure if I should do a special casing for Apple. It is 
# unlikely that other Unix systems will find the framework path.
# But if they do ([Next|Open|GNU]Step?), 
# do they want the -framework option also?
IF(${OPENTHREADS_INCLUDE_DIR_TMP} MATCHES ".framework")
	STRING(REGEX REPLACE "(.*)/.*\\.framework/.*" "\\1" OPENTHREADS_FRAMEWORK_PATH_TMP ${OPENTHREADS_INCLUDE_DIR_TMP})

	# To play nice so the INCLUDE_DIR is set like other platforms, I will 
	# just set a plausible value (though it really won't have an effect).
	# TODO: Figure out a way $ENV can be used.
	FIND_PATH(OPENTHREADS_INCLUDE_DIR OpenThreads.framework/Headers/Thread
		~/Library/Frameworks
		/Library/Frameworks
	)
	# It used to be true that /Libray/Frameworks and 
	# /System/Library/Frameworks was always searched
	# automatically, but there are changes due to the migration to 
	# Universal Binaries. To be safe, I will specify the -F flag regardless.
	# Unfortunately, it seems that this ADD_DEFINITIONS call is being ignored.
	# I think this is a CMake bug. Worst case, you must copy this line 
	# into your own scripts.
	ADD_DEFINITIONS(-F${OPENTHREADS_INCLUDE_DIR})

	# Set the library to use -framework
	SET(OPENTHREADS_LIBRARY "-F${OPENTHREADS_FRAMEWORK_PATH_TMP} -framework OpenThreads" CACHE STRING "OpenThreads framework")

	# Clear the temp variable so nobody can see it
	SET(OPENTHREADS_FRAMEWORK_PATH_TMP "" CACHE INTERNAL "")

ELSE(${OPENTHREADS_INCLUDE_DIR_TMP} MATCHES ".framework")
	FIND_LIBRARY(OPENTHREADS_LIBRARY_RELEASE
		NAMES OpenThreads OpenThreadsWin32
		PATHS
		$ENV{OPENTHREADSDIR}/lib
		$ENV{OPENTHREADSDIR}/lib/Win32
		${OPENTHREADSDIR}/lib
		${OPENTHREADSDIR}/lib/Win32
		$ENV{OSGDIR}/lib
		$ENV{OSGDIR}/lib/Win32
		${OSGDIR}/lib
		${OSGDIR}/lib/Win32
		/usr/local/lib
		/usr/lib
		/sw/lib
		/opt/local/lib
		/opt/csw/lib
		/opt/lib
	)
	FIND_LIBRARY(OPENTHREADS_LIBRARY_DEBUG
		NAMES OpenThreadsWin32d OpenThreadsd
		PATHS
		$ENV{OPENTHREADSDIR}/lib
		$ENV{OPENTHREADSDIR}/lib/Win32
		${OPENTHREADSDIR}/lib
		${OPENTHREADSDIR}/lib/Win32
		$ENV{OSGDIR}/lib
		$ENV{OSGDIR}/lib/Win32
		${OSGDIR}/lib
		${OSGDIR}/lib/Win32
		/usr/local/lib
		/usr/lib
		/sw/lib
		/opt/local/lib
		/opt/csw/lib
		/opt/lib
	)
	SET(OPENTHREADS_LIBRARY ${OPENTHREADS_LIBRARY_RELEASE})

ENDIF(${OPENTHREADS_INCLUDE_DIR_TMP} MATCHES ".framework")

# Clear the temp variable so nobody can see it
SET(OPENTHREADS_INCLUDE_DIR_TMP "" CACHE INTERNAL "")

SET(OPENTHREADS_FOUND "NO")
IF(OPENTHREADS_LIBRARY)
        SET(OPENTHREADS_FOUND "YES")
ENDIF(OPENTHREADS_LIBRARY)


