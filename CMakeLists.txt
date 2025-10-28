cmake_minimum_required(VERSION 3.16)

project(RevEngine VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
include(GNUInstallDirs)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt NAMES Qt6 REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS Gui)
find_package(Qt6 OPTIONAL_COMPONENTS OpenGL OpenGLWidgets Widgets)
find_package(PCL 1.15 REQUIRED COMPONENTS common filters kdtree features segmentation)
find_package(OpenMP)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

qt_add_executable(RevEngine
    ${PROJECT_SOURCES}
    Viewport.h Viewport.cpp
    data/Model.h data/Model.cpp


    ModelManager.h ModelManager.cpp
    Scene.h Scene.cpp
    renderers/Renderer.h renderers/Renderer.cpp
    Camera.h Camera.cpp
    renderers/ModelRenderer.h renderers/ModelRenderer.cpp
    Settings.h
    materials/Material.cpp materials/Material.h materials/Phong.cpp materials/Phong.h
    materials/PointCloudMaterial.h materials/PointCloudMaterial.cpp
    resources.qrc
    CADConverter.h CADConverter.cpp
    primitives/PrimitiveShape.cpp primitives/PrimitiveShape.h
    primitives/PrimitiveType.h
    primitives/Plane.h primitives/Plane.cpp
    materials/NormalsMaterial.h materials/NormalsMaterial.cpp
    dbscan/vendor/nanoflann/nanoflann.hpp dbscan/dbscan.cpp dbscan/dbscan.hpp  dbscan/LICENSE
    materials/WorldMaterial.h materials/WorldMaterial.cpp
    Util.h
    primitives/RenderShape.h
    materials/ShapeMaterial.h materials/ShapeMaterial.cpp
    primitives/Cylinder.h
    primitives/Cylinder.cpp
    primitives/Circle.h primitives/Circle.cpp
    data/BoundingBox.h
    data/ModelStatus.h
    exceptions/FileReadException.h
)

target_include_directories(RevEngine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(RevEngine
    PRIVATE
    Qt::Core
    Qt::Gui
    Qt::OpenGL
    Qt::OpenGLWidgets
    Qt::Widgets
    ${PCL_COMMON_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_FEATURES_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    # Eigen3::Eigen
)


if(OpenMP_CXX_FOUND)
    target_link_libraries(RevEngine
        PUBLIC
        OpenMP::OpenMP_CXX
    )
else()
    message(SEND_ERROR "OpenMP not found!")
endif()

# install(TARGETS RevEngine
#     BUNDLE DESTINATION .
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )
