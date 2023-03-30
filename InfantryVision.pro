TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

DEFINES += QT_DEPRECATED_WARNINGS

LIBS+=  -lpthread \
        -lX11
SOURCES += main.cpp \
    AngleSolver.cpp \
    ArmorBox.cpp \
    ArmorDeep.cpp \
    ArmorDetector.cpp \
    ArmorLight.cpp \
    FrameReceiver.cpp \
    SerialTest.cpp

INCLUDEPATH +=/usr/include/opencv4\
/opt/MVS/include

LIBS += /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so \
/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_core.so \
/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_dnn.so \
/usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_features2d.so \
/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_flann.so \
/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_gapi.so \
/usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_highgui.so \
/usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so \
/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so \
/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_ml.so \
/usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so \
/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_photo.so \
/usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_stitching.so \
/usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_video.so \
/usr/lib/aarch64-linux-gnu/libopencv_video.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_video.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libopencv_videoio.so \
/usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.1 \
/usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.1.1 \
/usr/lib/aarch64-linux-gnu/libpthread.so\
/usr/lib/aarch64-linux-gnu/libglib-2.0.so\
/usr/lib/aarch64-linux-gnu/libglib-2.0.so.0\
/opt/MVS/lib/aarch64/libMvCameraControl.so\
/usr/lib/libvisionworks_tracking.so\
/usr/lib/libvisionworks_tracking.so.0.88\
/usr/lib/libvisionworks_tracking.so.0.88.2

QMAKE_LFLAGS+= -no-pie

CONFIG+=console

HEADERS += \
    main.h \
    AngleSolver.h \
    ArmorBox.h \
    ArmorDeep.h \
    ArmorDetector.h \
    ArmorLight.h \
    FrameReceiver.h \
    SerialTest.h

FORMS += \
