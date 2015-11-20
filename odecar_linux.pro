TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += dDOUBLE
SOURCES += main.cpp \
    include/vmCar.cpp

INCLUDEPATH += /home/jlo/ode-0.13/include
INCLUDEPATH += /home/jlo/ode-0.13/drawstuff/src
INCLUDEPATH += /home/jlo/ode-0.13/ode/src

HEADERS += \
    include/vmCar.h

RC_FILE += /home/jlo/ode-0.13/drawstuff/src/resources.rc

win32:CONFIG(release, debug|release): LIBS += -L/home/jlo/ode-0.13/lib/DebugDoubleLib/ -ldrawstuff
else:win32:CONFIG(debug, debug|release): LIBS += -L/home/jlo/ode-0.13/lib/DebugDoubleLib/ -ldrawstuffd

INCLUDEPATH += /home/jlo/ode-0.13/lib/DebugDoubleLib
DEPENDPATH += /home/jlo/ode-0.13/lib/DebugDoubleLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/libdrawstuff.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/libdrawstuffd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/drawstuff.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/drawstuffd.lib



win32:CONFIG(release, debug|release): LIBS += -L/home/jlo/ode-0.13/lib/DebugDoubleLib/ -lode_double
else:win32:CONFIG(debug, debug|release): LIBS += -L/home/jlo/ode-0.13/lib/DebugDoubleLib/ -lode_doubled

INCLUDEPATH += /home/jlo/ode-0.13/lib/DebugDoubleLib
DEPENDPATH += /home/jlo/ode-0.13/lib/DebugDoubleLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/libode_double.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/libode_doubled.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/ode_double.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += /home/jlo/ode-0.13/lib/DebugDoubleLib/ode_doubled.lib


LIBS += -lode_doubled
LIBS += -ldrawstuffd
LIBS += -lGL
LIBS += -lGLU
LIBS += -lX11
LIBS += -lpthread
