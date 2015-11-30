#include "vmwishboncar.h"

vmWishboneCar::vmWishboneCar(dWorldID world, dSpaceID space)
    :vmCar(world, space)
{

}

vmWishboneCar::~vmWishboneCar()
{

}

void vmWishboneCar::setWheelJoint(vm::WheelLoc loc)
{
    // select wheel
    vmWheel *wnow;
    vmWishbone *snow;
    dReal shiftRatio;
    switch (loc) {
    case vm::WheelLoc::FR:
        snow= &frSuspension;
        wnow= &frWheel;
        shiftRatio= 0.5;
        break;
    case vm::WheelLoc::FL:
        snow= &flSuspension;
        wnow= &flWheel;
        shiftRatio= -0.5;
        break;
    case vm::WheelLoc::RR:
        snow= &rrSuspension;
        wnow= &rrWheel;
        shiftRatio= 0.5;
        break;
    case vm::WheelLoc::RL:
        snow= &rlSuspension;
        wnow= &rlWheel;
        shiftRatio= -0.5;
        break;
    default:
        break;
    }

    // set link dimension
    snow->uplink.mass= 1.0;
    snow->uplink.radius= 0.03;
    snow->uplink.width= 0.3;
    snow->hlink.mass= 0.5;
    snow->hlink.radius= 0.03;
    snow->hlink.width= 0.25;
    snow->lowlink.mass= 1.0;
    snow->lowlink.radius= 0.03;
    snow->lowlink.width= 0.4;

    // build linkage
    buildWheelJointLinkage(wnow,snow,shiftRatio);
    // build joint
    buildWheelJoint(wnow,snow,shiftRatio);

}

void vmWishboneCar::buildWheelJoint(vmWheel *wnow, vmWishbone *snow, dReal shiftRatio)
{
    // chassis pin joints
    const dReal *pos;
    snow->jChassisUp= dJointCreateHinge(world,0);
    dJointAttach(snow->jChassisUp,chassis.body,snow->uplink.body);
    dJointSetHingeAxis(snow->jChassisUp,1.0, 0.0, 0.0);
    pos= dBodyGetPosition(snow->uplink.body);
    dJointSetHingeAnchor(snow->jChassisUp,pos[0], pos[1]+shiftRatio*snow->uplink.width, pos[2]);

    const dReal *pos1;
    snow->jChassisLow= dJointCreateHinge(world,0);
    dJointAttach(snow->jChassisLow, chassis.body, snow->lowlink.body);
    dJointSetHingeAxis(snow->jChassisLow,1.0, 0.0, 0.0);
    pos1= dBodyGetPosition(snow->lowlink.body);
    dJointSetHingeAnchor(snow->jChassisLow, pos1[0], pos1[1]+shiftRatio*snow->lowlink.width, pos1[2]);

    // rotor ball joint
    /*const dReal *p2;
    jRotorUp= dJointCreateBall(world,0);
    dJointAttach(jRotorUp, uplink.body, hlink.body);
    p2= dBodyGetPosition(uplink.body);
    dJointSetBallAnchor(jRotorUp, p2[0], p2[1]-0.5*uplink.width, p2[2]);

    const dReal *p3;
    jRotorLow= dJointCreateBall(world,0);
    dJointAttach(jRotorLow, lowlink.body,hlink.body);
    p3= dBodyGetPosition(lowlink.body);
    dJointSetBallAnchor(jRotorLow, p3[0], p3[1]-0.5*lowlink.width,p3[2]);*/

    const dReal *p2;
    snow->jRotorUp= dJointCreateHinge(world,0);
    dJointAttach(snow->jRotorUp, snow->uplink.body, snow->hlink.body);
    p2= dBodyGetPosition(snow->uplink.body);
    dJointSetHingeAnchor(snow->jRotorUp, p2[0], p2[1]-shiftRatio*snow->uplink.width, p2[2]);
    dJointSetHingeAxis(snow->jRotorUp, 1.0, 0.0, 0.0);

    const dReal *p3;
    snow->jRotorLow= dJointCreateHinge(world,0);
    dJointAttach(snow->jRotorLow, snow->lowlink.body,snow->hlink.body);
    p3= dBodyGetPosition(snow->lowlink.body);
    dJointSetHingeAnchor(snow->jRotorLow, p3[0], p3[1]-shiftRatio*snow->lowlink.width,p3[2]);
    dJointSetHingeAxis(snow->jRotorLow, 1.0, 0.0, 0.0);

    // rotor hinge joint
    snow->jRotorMid= dJointCreateHinge(world,0);
    dJointAttach(snow->jRotorMid, snow->hlink.body, wnow->body);
    dJointSetHingeAxis(snow->jRotorMid, 0.0,1.0,0.0);
    dJointSetHingeAnchor(snow->jRotorMid, 0.0, 0.0, wnow->radius);

    // suspension slider
    snow->jLowSpring= dJointCreateLMotor(world,0);
    dJointAttach(snow->jLowSpring, chassis.body, snow->lowlink.body);
    dJointSetLMotorNumAxes(snow->jLowSpring, 1);
    dJointSetLMotorAxis(snow->jLowSpring,0,1, 0.0, 0.0, 1.0);
}

void vmWishboneCar::setWheelSuspension(vm::WheelLoc loc, dReal step, dReal kps, dReal kds)
{
    vmWheel *wnow;
    vmWishbone *snow;
    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        snow= &frSuspension;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        snow= &flSuspension;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        snow= &rrSuspension;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        snow= &rlSuspension;
        break;
    default:
        break;
    }

    dJointSetHinge2Param(wnow->joint,dParamSuspensionERP, computeERP(step,kps,kds)); // suspension
    dJointSetHinge2Param(wnow->joint,dParamSuspensionCFM, computeCFM(step,kps,kds)); // suspension

    dJointSetLMotorParam(snow->jLowSpring, dParamHiStop, 0.0);
    dJointSetLMotorParam(snow->jLowSpring, dParamLoStop, 0.0);
    dJointSetLMotorParam(snow->jLowSpring, dParamStopCFM, computeCFM(step,kps,kds));
    dJointSetLMotorParam(snow->jLowSpring, dParamStopERP, computeERP(step,kps,kds));
    dJointSetLMotorParam(snow->jLowSpring, dParamFMax, 1000);
}

void vmWishboneCar::buildWheelJointLinkage(vmWheel *wnow, vmWishbone *snow, dReal shiftRatio)
{
    // some parameters
    dMass m1;
    dMatrix3 rmat;
    dReal *wheelPos= dBodyGetPosition(wnow);

    // uplink
    snow->uplink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->uplink.mass,3
                          ,snow->uplink.radius
                          ,snow->uplink.width);
    dBodySetMass(snow->uplink.body,&m1);
    dBodySetPosition(snow->uplink.body, wheelPos[0]
            ,wheelPos[1]+shiftRatio*(wnow->length+snow->uplink.width)
            ,wheelPos[2]+0.5*snow->hlink.width);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 90*M_PI/180.0);
    dBodySetRotation(snow->uplink.body,rmat);

    // hlink
    snow->hlink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->hlink.mass,3
                          ,snow->hlink.radius
                          ,snow->hlink.width);
    dBodySetMass(snow->hlink.body,&m1);
    dBodySetPosition(snow->hlink.body, wheelPos[0]
            ,wheelPos[1],wheelPos[2]);

    // lowlink
    snow->lowlink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->lowlink.mass,3
                          ,snow->lowlink.radius
                          ,snow->lowlink.width);
    dBodySetMass(snow->lowlink.body,&m1);
    dBodySetPosition(snow->lowlink.body, wheelPos[0]
            ,wheelPos[1]+shiftRatio(wnow->length+snow->lowlink.width)
            ,wheelPos[2]-0.5*snow->hlink.width);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 90*M_PI/180.0);
    dBodySetRotation(snow->lowlink.body,rmat);
}

