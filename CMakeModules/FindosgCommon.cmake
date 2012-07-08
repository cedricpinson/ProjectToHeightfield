# This is part of the Findosg* suite used to find OpenSceneGraph components.
# Each component is separate and you must opt in to each module. You must 
# also opt into OpenGL and OpenThreads (and Producer if needed) as these 
# modules won't do it for you. This is to allow you control over your own 
# system piece by piece in case you need to opt out of certain components
# or change the Find behavior for a particular module (perhaps because the
# default FindOpenGL.cmake module doesn't work with your system as an
# example).
# If you want to use a more convenient module that includes everything,
# use the FindOpenSceneGraph.cmake instead of the Findosg*.cmake modules.
# 
# Locate osgParticle
# This module defines
# FINDOSGCOMMON
#
# IncludeFileTest is a Header file used to find the INCLUDE_DIR
# For Example: if file to used is osg/Referenced, pass "Referenced" to IncludeFileTest
#
# OsgNamespace is the namespace of the library
# For Example: if you search the osgIntrospection Library, pass "osgIntrospection" to OsgNamespace
#
# $OSGDIR is an environment variable that would
# correspond to the ./configure --prefix=$OSGDIR
# used in building osg.
#
# Created by David Callu
# Tested by Laurent Marchal
# Based on Eric Wing Module





MACRO(FINDOSGCOMMON IncludeFileTest OsgNamespace)

  STRING(TOUPPER ${OsgNamespace} OSGNAMESPACE)

  # Header files are presumed to be included like
  # #include <osg/PositionAttitudeTransform>

  # Try the user's environment request before anything else.
  FIND_PATH(${OSGNAMESPACE}_INCLUDE_DIR ${OsgNamespace}/${IncludeFileTest}
	$ENV{OSGDIR}/include
	${OSGDIR}/include
    )

  # On OSX, this will prefer the Framework version (if found) over others
  # except when the environment is set to a valid location.
  # People will have to manually change the cache values of 
  # OSG${OsgNamespace}_LIBRARY to override this selection.
  # OS X is a nasty corner case because CMake's FIND_PATH isn't framework aware.
  # Apple provides magic so that #include <osg/PositionAttitudeTransform>
  # maps to */Library/Frameworks/osg.framework/Headers/PositionAttitudeTransform
  # but CMake FIND_PATH isn't smart enough to handle this if I specify the 
  # file as osg/PositionAttitudeTransform since no osg/PositionAttitudeTransform
  # actually exists.
  # But it gets worse. I actually don't want the path mapped to the Headers 
  # directory, but only to the directory that holds the framework (e.g 
  # /Library/Frameworks). In the past, OS X automatically searched for
  # the /Library/Frameworks directories, but with Universal Binaries, the 
  # process has changed. Furthermore, it won't cover cases where the frameworks
  # are located elsewhere.
  IF(NOT ${OSGNAMESPACE}_INCLUDE_DIR)
    # For usage of #include <Foo/foo.h> with a framework, the FIND_PATH 
    # and traditional -I parameter is useless. The -F parameter must be
    # used. So we will use FIND_PATH to a temporary variable just so 
    # we know if we have a framework and where it is.
    # TODO: Figure out a way $ENV can be used.
    FIND_PATH(${OSGNAMESPACE}_INCLUDE_DIR_TMP IncludeFileTest
      ~/Library/Frameworks/${OsgNamespace}.framework/Headers
      /Library/Frameworks/${OsgNamespace}.framework/Headers
      )
    
    # If the Framework style is not found, do the classic approach
    IF(NOT ${OSGNAMESPACE}_INCLUDE_DIR_TMP)
      FIND_PATH(${OSGNAMESPACE}_INCLUDE_DIR ${OsgNamespace}/${IncludeFileTest}
	/usr/local/include
	/usr/include
	/sw/include # Fink
	/opt/local/include # DarwinPorts
	/opt/csw/include # Blastwave
	/opt/include
	)
    ENDIF(NOT ${OSGNAMESPACE}_INCLUDE_DIR_TMP)
  ENDIF(NOT ${OSGNAMESPACE}_INCLUDE_DIR)
  
  
  # I'm not sure if I should do a special casing for Apple. It is 
  # unlikely that other Unix systems will find the framework path.
  # But if they do ([Next|Open|GNU]Step?), 
  # do they want the -framework option also?
  IF(${${OSGNAMESPACE}_INCLUDE_DIR_TMP} MATCHES ".framework")
    STRING(REGEX REPLACE "(.*)/.*\\.framework/.*" "\\1" ${OSGNAMESPACE}_FRAMEWORK_PATH_TMP ${${OSGNAMESPACE}_INCLUDE_DIR_TMP})
    
    # To play nice so the INCLUDE_DIR is set like other platforms, I will 
    # just set a plausible value (though it really won't have an effect).
    # TODO: Figure out a way $ENV can be used.
    FIND_PATH(${OSGNAMESPACE}_INCLUDE_DIR ${OsgNamespace}.framework/Headers/${IncludeFileTest}
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
    ADD_DEFINITIONS(-F${${OSGNAMESPACE}_INCLUDE_DIR})
    
    # Set the library to use -framework
    SET(${OSGNAMESPACE}_LIBRARY "-F${${OSGNAMESPACE}_FRAMEWORK_PATH_TMP} -framework ${OsgNamespace}" CACHE STRING "${OsgNamespace} framework")
    
    # Clear the temp variable so nobody can see it
    SET(${OSGNAMESPACE}_FRAMEWORK_PATH_TMP "" CACHE INTERNAL "")
    
  ELSE(${${OSGNAMESPACE}_INCLUDE_DIR_TMP} MATCHES ".framework")
    FIND_LIBRARY(${OSGNAMESPACE}_LIBRARY_RELEASE
      NAMES ${OsgNamespace}
      PATHS
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
    FIND_LIBRARY(${OSGNAMESPACE}_LIBRARY_DEBUG
      NAMES ${OsgNamespace}d ${OsgNamespace}
      PATHS
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
    SET(${OSGNAMESPACE}_LIBRARY ${${OSGNAMESPACE}_LIBRARY_RELEASE})
  ENDIF(${${OSGNAMESPACE}_INCLUDE_DIR_TMP} MATCHES ".framework")
  
  # Clear the temp variable so nobody can see it
  SET(${OSGNAMESPACE}_INCLUDE_DIR_TMP "" CACHE INTERNAL "")
  
  SET(${OSGNAMESPACE}_FOUND "NO")
  IF(${OSGNAMESPACE}_LIBRARY)
    SET(${OSGNAMESPACE}_FOUND "YES")
  ENDIF(${OSGNAMESPACE}_LIBRARY)
  
ENDMACRO(FINDOSGCOMMON)