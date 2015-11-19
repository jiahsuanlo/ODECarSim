TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += dDOUBLE
SOURCES += main.cpp \
    include/vmCar.cpp

INCLUDEPATH += "C:/dev/ode-0.13/include"
INCLUDEPATH += "C:/dev/ode-0.13/drawstuff/src"
INCLUDEPATH += "C:/dev/ode-0.13/ode/src"

HEADERS += \
    include/vmCar.h

RC_FILE += "C:\dev\ode-0.13\drawstuff\src\resources.rc"

win32:CONFIG(release, debug|release): LIBS += -LC:/dev/ode-0.13/lib/DebugDoubleLib/ -ldrawstuff
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/dev/ode-0.13/lib/DebugDoubleLib/ -ldrawstuffd

INCLUDEPATH += C:/dev/ode-0.13/lib/DebugDoubleLib
DEPENDPATH += C:/dev/ode-0.13/lib/DebugDoubleLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += C:./dev/ode-0.13/lib/DebugDoubleLib/libdrawstuff.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/libdrawstuffd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/drawstuff.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/drawstuffd.lib



win32:CONFIG(release, debug|release): LIBS += -LC:/dev/ode-0.13/lib/DebugDoubleLib/ -lode_double
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/dev/ode-0.13/lib/DebugDoubleLib/ -lode_doubled

INCLUDEPATH += C:/dev/ode-0.13/lib/DebugDoubleLib
DEPENDPATH += C:/dev/ode-0.13/lib/DebugDoubleLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/libode_double.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/libode_doubled.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/ode_double.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += C:/dev/ode-0.13/lib/DebugDoubleLib/ode_doubled.lib


LIBS += -lode_doubled
LIBS += -ldrawstuffd
LIBS += -luser32
LIBS += -lwinmm
LIBS += -lgdi32
LIBS += -lopengl32
LIBS += -lglu32
