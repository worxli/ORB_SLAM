QT += core
QT -= gui

TARGET = ORB_SLAM_Qt
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += \
    $$PWD/include \
    /opt/ros/jade/include

SOURCES += \
    src/Converter.cc \
    src/Frame.cc \
    src/FramePublisher.cc \
    src/Initializer.cc \
    src/KeyFrame.cc \
    src/KeyFrameDatabase.cc \
    src/LocalMapping.cc \
    src/LoopClosing.cc \
    src/main.cc \
    src/Map.cc \
    src/MapPoint.cc \
    src/MapPublisher.cc \
    src/Optimizer.cc \
    src/ORBextractor.cc \
    src/ORBmatcher.cc \
    src/PnPsolver.cc \
    src/Sim3Solver.cc \
    src/Tracking.cc

HEADERS += \
    include/Converter.h \
    include/Frame.h \
    include/FramePublisher.h \
    include/Initializer.h \
    include/KeyFrame.h \
    include/KeyFrameDatabase.h \
    include/LocalMapping.h \
    include/LoopClosing.h \
    include/Map.h \
    include/MapPoint.h \
    include/MapPublisher.h \
    include/Optimizer.h \
    include/ORBextractor.h \
    include/ORBmatcher.h \
    include/ORBVocabulary.h \
    include/PnPsolver.h \
    include/Sim3Solver.h \
    include/Tracking.h

