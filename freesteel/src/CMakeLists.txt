
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

add_definitions(-Wall)

add_library(actp STATIC
    bolts/maybe.h
    bolts/debugfuncs.h
    bolts/I1.h
    bolts/P2.h
    bolts/P3.h
    bolts/Partition1.cpp
    bolts/Partition1.h
    bolts/S1.cpp
    bolts/S1.h
    bolts/smallfuncs.h
    bolts/vo.h
    cages/Area2_gen.cpp
    cages/Area2_gen.h
    cages/PathXboxed.cpp
    cages/PathXboxed.h
    cages/Ray_gen2.cpp
    cages/Ray_gen2.h
    cages/PathXSeries.h
    cages/S2weave.cpp
    cages/S2weave.h
    cages/SurfXboxed.cpp
    cages/SurfXboxed.h
    cages/SurfXBuildComponents.cpp
    cages/SurfX.cpp
    cages/SurfX.h
    pits/CircCrossingStructure.h
    pits/CircCrossingStructure.cpp
    pits/CoreRoughGeneration.h
    pits/CoreRoughGeneration.cpp
    pits/NormRay_gen.cpp
    pits/NormRay_gen.h
    pits/S2weaveCell.cpp
    pits/S2weaveCell.h
    pits/S2weaveCellLinearCut.cpp
    pits/S2weaveCellLinearCut.h
    pits/S2weaveCellLinearCutTraverse.cpp
    pits/SLi_gen.cpp
    pits/SLi_gen.h
)

#target_link_libraries(actp )

add_executable(canary canary.cpp)
target_link_libraries(canary actp)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(wxWidgets REQUIRED gl core base)
include(${wxWidgets_USE_FILE})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

FIND_PACKAGE(PkgConfig)
pkg_check_modules (GTK2 gtk+-2.0)
INCLUDE_DIRECTORIES(${GTK2_INCLUDE_DIRS})

add_executable(freesteel
    visuals/fsvtkToolpathMapper.cpp
    visuals/fsvtkToolpathMapper.h
    visuals/GeometryStuffTogether.cpp
    visuals/gst.h
    visuals/gstsurface.cpp
    visuals/gstsurface.h
    visuals/toolshape.cpp
    visuals/toolshape.h
    xenon/AnimationFrame.cpp
    xenon/AnimationFrame.h
    xenon/BoundingBoxDlg.cpp
    xenon/BoundingBoxDlg.h
    xenon/coreroughdlg.cpp
    xenon/coreroughdlg.h
    xenon/MainFrame.cpp
    xenon/MainFrame.h
    xenon/MyApp.cpp
    xenon/MyApp.h
    xenon/wxhelpers.cpp
    xenon/wxhelpers.h
    xenon/wxVTKRenderWindowInteractor.cxx
    xenon/wxVTKRenderWindowInteractor.h    
)

target_link_libraries(freesteel
    ${wxWidgets_LIBRARIES}
    ${GTK2_LIBRARIES}
    ${OPENGL_LIBRARY}
    ${VTK_LIBRARIES}
    actp
)


