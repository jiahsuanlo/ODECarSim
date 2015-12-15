#ifndef VMWISHBONECAR_H
#define VMWISHBONECAR_H

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
    cylinder uplink{0.0,0.0,0.0};
    cylinder lowlink{0.0,0.0,0.0};
    cylinder hlink{0.0,0.0,0.0};
    cylinder upstrut{0.0,0.0,0.0};
    cylinder lowstrut{0.0,0.0,0.0};
    dJointID jChassisUp,jChassisLow;
    dJointID jRotorUp, jRotorLow, jRotorMid;
    dJointID jStrutUp, jStrutLow, jStrutMid;
};

class vmWishboneCar : public vmCar
{
public:
    vmWishboneCar(dWorldID world,dSpaceID space): vmCar(world,space){}
    virtual ~vmWishboneCar();

    // member
    vmWishbone frSuspension;
    vmWishbone flSuspension;
    vmWishbone rrSuspension;
    vmWishbone rlSuspension;

    dReal strutAngle= 5*M_PI/180.0;

    // setter
    virtual void setWheelJoint(vm::WheelLoc loc);
    void buildWheelJointLinkage(vmWheel *wnow, vmWishbone *snow, dReal shiftSign);
    void buildWheelJoint(vmWheel *wnow, vmWishbone *snow, dReal shiftSign);

    virtual void setWheelSuspension(vm::WheelLoc loc,dReal step,dReal kps, dReal kds);

    // simloop functions
    virtual void simControl();
    virtual void simForward(dReal mphSpeed);
    //virtual void simSlowSteer();

    // simloop draw function
    virtual void simDraw();
    void drawLinkage(vm::WheelLoc loc);

    // suspension functions
    virtual dReal getSuspensionRate(vm::WheelLoc loc, dReal step);

};

#endif // VMWISHBONECAR_H
