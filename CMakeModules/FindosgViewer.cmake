GET_FILENAME_COMPONENT(CURRENT_PATH ${CMAKE_CURRENT_LIST_FILE} PATH)

INCLUDE(${CURRENT_PATH}/FindosgCommon.cmake)

FINDOSGCOMMON("View" "osgViewer")
