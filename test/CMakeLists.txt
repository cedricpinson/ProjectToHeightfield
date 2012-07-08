SET(TEST_SRC
  test_projection.cpp
  ../ProjectorVisitor.cpp 
 )

ENABLE_TESTING()
ADD_TEST(TestProjection TestProjection)
FIND_PACKAGE(UnitTest++)
INCLUDE_DIRECTORIES(${UNITTEST++_INCLUDE_DIR})
ADD_EXECUTABLE(TestProjection ${TEST_SRC})
ADD_DEFINITIONS(-DTEST_DIR=\\"${ProjectorVisitorExample_SOURCE_DIR}/test/\\" )
TARGET_LINK_LIBRARIES(TestProjection ${LIBS} ${UNITTEST++_LIBRARY})
