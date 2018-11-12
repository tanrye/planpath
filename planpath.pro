TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    coordinate_tf.cpp \
    least_squares.cpp \
    automatching.cpp \
    plan_path.cpp \
    rosmsgcallback.cpp

HEADERS += \
    coordinate_tf.h \
    rosmessage.h \
    spline.h \
    least_squares.h \
    trackpoint.h \
    lane_point.h \
    local_planning.h \
    path_data.h \
    ins_p2.h

INCLUDEPATH += /opt/ros/kinetic/include/ \
               /opt/ros/kinetic/include \
    ../include \
    ../src

LIBS += -L/opt/ros/kinetic/lib \
-lroscpp \
-lrosconsole \
-lroscpp_serialization \
-lrostime \
-lboost_system \
-lboost_thread \
-lpthread \
-lxmlrpcpp \
-lcpp_common \
-lrosconsole_log4cxx \
-lrosconsole_backend_interface \
