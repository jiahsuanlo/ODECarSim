#ifndef VMWISHBONCAR_H
#define VMWISHBONCAR_H

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "vmCar.h"

struct cylinder{
    dBodyID body;
    dGeomID geom;
    dReal radius;
    dReal width;
    dReal mass;
    cylinder(dReal mass,dReal radius, dReal width):
        mass{mass}, radius{radius}, width{width}{}
};

struct vmWishbone{
    cylinder uplink;
    cylinder lowlink;
    cylinder hlink;
    dJointID jChassisUp,jChassisLow;
    dJointID jRotorUp, jRotorLow, jRotorMid;
    dJointID jLowSpring;
};

class vmWishbonCar : public vmCar
{
public:
    vmWishbonCar(dWorldID world, dSpaceID space);
    virtual ~vmWishboneCar();

    // member
    vmWishbone frSuspension;
    vmWishbone flSuspension;
    vmWishbone rrSuspension;
    vmWishbone rlSuspension;

    // setter
    virtual void setWheelJoint(vm::WheelLoc loc);

};

#endif // VMWISHBONCAR_H
