cmake_minimum_required(VERSION 3.16)

project(RevEngine VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGLWidgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS OpenGL)
find_package(PCL 1.15 REQUIRED)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(RevEngine
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        Viewport.h Viewport.cpp
        data/Model.h data/Model.cpp
        data/Mesh.h data/Mesh.cpp
        data/PointCloud.h data/PointCloud.cpp
        ModelManager.h ModelManager.cpp
        Scene.h Scene.cpp
        renderers/Renderer.h renderers/Renderer.cpp
        Camera.h Camera.cpp
        renderers/ModelRenderer.h renderers/ModelRenderer.cpp
        Settings.h Settings.cpp
        resources/shaders/pointcloud.vert
        resources/shaders/pointcloud.frag
        materials/Material.cpp materials/Material.h materials/Phong.cpp materials/Phong.h materials/Wireframe.cpp materials/Wireframe.h
        materials/PointCloudMaterial.h materials/PointCloudMaterial.cpp
        resources.qrc
        CADConverter.h CADConverter.cpp
        HoughTransformer.h HoughTransformer.cpp
        primitives/PrimitiveShape.cpp primitives/PrimitiveShape.h
        primitives/PrimitiveType.h
        primitives/NormalPlane.h primitives/NormalPlane.cpp



    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET RevEngine APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(RevEngine SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(RevEngine
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(RevEngine
    PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    ${PCL_COMMON_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES})

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.RevEngine)
endif()
set_target_properties(RevEngine PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS RevEngine
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(RevEngine)
endif()
