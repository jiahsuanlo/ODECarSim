TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

#DEFINES += dDOUBLE
SOURCES += main.cpp \
    include/vmCar.cpp

INCLUDEPATH += /home/jlo/ode-0.13/include
INCLUDEPATH += /home/jlo/ode-0.13/drawstuff/src
INCLUDEPATH += /home/jlo/ode-0.13/ode/src

HEADERS += \
    include/vmCar.h

RC_FILE += /home/jlo/ode-0.13/drawstuff/src/resources.rc


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../ode-0.13/lib/DebugDoubleLib/release/ -ldrawstuffd
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../ode-0.13/lib/DebugDoubleLib/debug/ -ldrawstuffd
else:unix: LIBS += -L$$PWD/../../../ode-0.13/lib/DebugDoubleLib/ -ldrawstuffd

INCLUDEPATH += $$PWD/../../../ode-0.13/lib/DebugDoubleLib
DEPENDPATH += $$PWD/../../../ode-0.13/lib/DebugDoubleLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/release/libdrawstuffd.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/debug/libdrawstuffd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/release/drawstuffd.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/debug/drawstuffd.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/libdrawstuffd.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../ode-0.13/lib/DebugDoubleLib/release/ -lode_doubled
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../ode-0.13/lib/DebugDoubleLib/debug/ -lode_doubled
else:unix: LIBS += -L$$PWD/../../../ode-0.13/lib/DebugDoubleLib/ -lode_doubled

INCLUDEPATH += $$PWD/../../../ode-0.13/lib/DebugDoubleLib
DEPENDPATH += $$PWD/../../../ode-0.13/lib/DebugDoubleLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/release/libode_doubled.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/debug/libode_doubled.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/release/ode_doubled.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/debug/ode_doubled.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/DebugDoubleLib/libode_doubled.a


unix:!macx: LIBS += -L$$PWD/../../../ode-0.13/lib/ReleaseDoubleLib/ -ldrawstuff

INCLUDEPATH += $$PWD/../../../ode-0.13/lib/ReleaseDoubleLib
DEPENDPATH += $$PWD/../../../ode-0.13/lib/ReleaseDoubleLib

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/ReleaseDoubleLib/libdrawstuff.a

unix:!macx: LIBS += -L$$PWD/../../../ode-0.13/lib/ReleaseDoubleLib/ -lode_double

INCLUDEPATH += $$PWD/../../../ode-0.13/lib/ReleaseDoubleLib
DEPENDPATH += $$PWD/../../../ode-0.13/lib/ReleaseDoubleLib

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../ode-0.13/lib/ReleaseDoubleLib/libode_double.a




LIBS += -lode_doubled
LIBS += -ldrawstuffd
LIBS += -lGL
LIBS += -lGLU
LIBS += -lX11
LIBS += -lpthread
