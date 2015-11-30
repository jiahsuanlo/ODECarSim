#include "vmwishboncar.h"

vmWishbonCar::vmWishbonCar(dWorldID world, dSpaceID space)
{
    this->world= world;
    this->space= space;
}

vmWishbonCar::~vmWishboneCar()
{

}

void vmWishbonCar::setWheelJoint(vm::WheelLoc loc)
{
    // select wheel
    vmWishbone *snow;
    switch (loc) {
    case vm::WheelLoc::FR:
        snow= &frSuspension;
        break;
    case vm::WheelLoc::FL:
        snow= &flSuspension;
        break;
    case vm::WheelLoc::RR:
        snow= &rrSuspension;
        break;
    case vm::WheelLoc::RL:
        snow= &rlSuspension;
        break;
    default:
        break;
    }

    dMass m1;
    dMatrix3 rmat;

    // uplink
    snow->uplink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->uplink.mass,3
                          ,snow->uplink.radius
                          ,snow->uplink.width);
    dBodySetMass(snow->uplink.body,&m1);

    /*
     * I am here !!!!!
     * Need to setup the uplink location
     */

    dBodySetPosition(snow->uplink.body, 0.0, 0.5*(frWheel.width+uplink.width),
                     1.0*frWheel.radius+0.5*hlink.width);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 90*M_PI/180.0);
    dBodySetRotation(uplink.body,rmat);

    // hlink
    hlink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,hlink.mass,3,hlink.radius,hlink.width);
    dBodySetMass(hlink.body,&m1);
    dBodySetPosition(hlink.body, 0.0, 0.5*frWheel.width, 1.0*frWheel.radius);

    // lowlink
    lowlink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,lowlink.mass,3,lowlink.radius,lowlink.width);
    dBodySetMass(lowlink.body,&m1);
    dBodySetPosition(lowlink.body, 0.0, 0.5*(frWheel.width+lowlink.width)
                     , 1.0*frWheel.radius-0.5*hlink.width);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 90*M_PI/180.0);
    dBodySetRotation(lowlink.body,rmat);

}

