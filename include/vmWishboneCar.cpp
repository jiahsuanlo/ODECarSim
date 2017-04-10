#include "vmWishboneCar.h"
vmWishboneCar::~vmWishboneCar()
{

}

void vmWishboneCar::setWheelJoint(vm::WheelLoc loc)
{
    // select wheel
    vmWheel *wnow= nullptr;
    vmWishbone *snow= nullptr;
    dReal shiftSign;
    switch (loc) {
    case vm::WheelLoc::FR:
        snow= &frSuspension;
        wnow= &frWheel;
        shiftSign= 1.0;
        break;
    case vm::WheelLoc::FL:
        snow= &flSuspension;
        wnow= &flWheel;
        shiftSign= -1.0;
        break;
    case vm::WheelLoc::RR:
        snow= &rrSuspension;
        wnow= &rrWheel;
        shiftSign= 1.0;
        break;
    case vm::WheelLoc::RL:
        snow= &rlSuspension;
        wnow= &rlWheel;
        shiftSign= -1.0;
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
    // setup sturt dimension
    snow->upstrut.mass= 2.0;
    snow->upstrut.radius= 0.03;
    snow->upstrut.width= 0.15;
    snow->lowstrut.mass= 2.0;
    snow->lowstrut.radius= 0.03;
    snow->lowstrut.width= 0.15;


    // build linkage
    buildWheelJointLinkage(wnow,snow,shiftSign);
    // build joint
    buildWheelJoint(wnow,snow,shiftSign);

}

void vmWishboneCar::buildWheelJoint(vmWheel *wnow, vmWishbone *snow, dReal shiftSign)
{
    // chassis pin joints
    const dReal *pos;
    snow->jChassisUp= dJointCreateHinge(world,0);
    dJointAttach(snow->jChassisUp,chassis.body,snow->uplink.body);
    dJointSetHingeAxis(snow->jChassisUp,1.0, 0.0, 0.0);
    pos= dBodyGetPosition(snow->uplink.body);
    dJointSetHingeAnchor(snow->jChassisUp,pos[0]
            ,pos[1]+0.5*shiftSign*snow->uplink.width, pos[2]);

    const dReal *pos1;
    snow->jChassisLow= dJointCreateHinge(world,0);
    dJointAttach(snow->jChassisLow, chassis.body, snow->lowlink.body);
    dJointSetHingeAxis(snow->jChassisLow,1.0, 0.0, 0.0);
    pos1= dBodyGetPosition(snow->lowlink.body);
    dJointSetHingeAnchor(snow->jChassisLow, pos1[0]
            , pos1[1]+0.5*shiftSign*snow->lowlink.width, pos1[2]);

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
    dJointSetHingeAnchor(snow->jRotorUp, p2[0]
            ,p2[1]-0.5*shiftSign*snow->uplink.width, p2[2]);
    dJointSetHingeAxis(snow->jRotorUp, 1.0, 0.0, 0.0);

    const dReal *p3;
    snow->jRotorLow= dJointCreateHinge(world,0);
    dJointAttach(snow->jRotorLow, snow->lowlink.body,snow->hlink.body);
    p3= dBodyGetPosition(snow->lowlink.body);
    dJointSetHingeAnchor(snow->jRotorLow, p3[0]
            ,p3[1]-0.5*shiftSign*snow->lowlink.width,p3[2]);
    dJointSetHingeAxis(snow->jRotorLow, 1.0, 0.0, 0.0);

    // rotor hinge joint
    const dReal *pw= dBodyGetPosition(wnow->body);
    snow->jRotorMid= dJointCreateHinge(world,0);
    dJointAttach(snow->jRotorMid, snow->hlink.body, wnow->body);
    dJointSetHingeAxis(snow->jRotorMid, 0.0,1.0,0.0);
    dJointSetHingeAnchor(snow->jRotorMid, pw[0],pw[1],pw[2]);


    // strut joint
    const dReal *ps1, *ps2;
    dReal angle= -shiftSign*strutAngle;

    ps1= dBodyGetPosition(snow->upstrut.body);
    ps2= dBodyGetPosition(snow->lowstrut.body);
    snow->jStrutUp= dJointCreateBall(world,0);
    dJointAttach(snow->jStrutUp, chassis.body, snow->upstrut.body);
    //dJointSetFixed(snow->jStrutUp);
    dJointSetBallAnchor(snow->jStrutUp, ps1[0]
            ,ps1[1]+0.5*shiftSign*snow->upstrut.width*fabs(sin(angle))
            ,ps1[2]+0.5*snow->upstrut.width*fabs(cos(angle)));

    snow->jStrutLow= dJointCreateBall(world,0);
    dJointAttach(snow->jStrutLow, snow->lowlink.body, snow->lowstrut.body);
    dJointSetBallAnchor(snow->jStrutLow, ps2[0]
            ,ps2[1]-0.5*shiftSign*snow->lowstrut.width*fabs(sin(angle))
            ,ps2[2]-0.5*snow->lowstrut.width*fabs(cos(angle)));

    // struct sliding joint
    snow->jStrutMid= dJointCreateSlider(world,0);
    dJointAttach(snow->jStrutMid, snow->upstrut.body, snow->lowstrut.body);
    dJointSetSliderAxis(snow->jStrutMid, 0.0,shiftSign*fabs(sin(angle)),fabs(cos(angle)));

    // set joint feedback
    wnow->feedback= new dJointFeedback;
    dJointSetFeedback(snow->jStrutMid,wnow->feedback);

    // suspension slider
    /*snow->jLowSpring= dJointCreateLMotor(world,0);
    dJointAttach(snow->jLowSpring, chassis.body, snow->lowlink.body);
    dJointSetLMotorNumAxes(snow->jLowSpring, 1);
    dJointSetLMotorAxis(snow->jLowSpring,0,0, 0.0, 0.0, 1.0);*/


}

void vmWishboneCar::setWheelSuspension(vm::WheelLoc loc, dReal step, dReal kps, dReal kds)
{
    vmWheel *wnow= nullptr;
    vmWishbone *snow= nullptr;
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

    dJointSetSliderParam(snow->jStrutMid, dParamHiStop, 0.0);
    dJointSetSliderParam(snow->jStrutMid, dParamLoStop, 0.0);
    dJointSetSliderParam(snow->jStrutMid, dParamStopCFM, computeCFM(step,kps,kds));
    dJointSetSliderParam(snow->jStrutMid, dParamStopERP, computeERP(step,kps,kds));
    dJointSetSliderParam(snow->jStrutMid, dParamFMax, dInfinity);

    /*dJointSetLMotorParam(snow->jLowSpring, dParamHiStop, 0.0);
    dJointSetLMotorParam(snow->jLowSpring, dParamLoStop, 0.0);
    dJointSetLMotorParam(snow->jLowSpring, dParamStopCFM, computeCFM(step,kps,kds));
    dJointSetLMotorParam(snow->jLowSpring, dParamStopERP, computeERP(step,kps,kds));
    dJointSetLMotorParam(snow->jLowSpring, dParamFMax, dInfinity);*/
}

void vmWishboneCar::simControl()
{
    // auto align steering angle
    if (manualYes)
        steerGain= 10;
    else
    {
        steerGain= 10;
        steer*= 0.98;
        speed*= 0.999;
    }


    dReal dsteer, realSpeed;
    //dsteer = bounded(steer,-40.0,40.0) - dJointGetHingeAngle(frWheel.joint);
    realSpeed = -bounded(speed, -14*M_PI, 30*M_PI);
    // steer
    /*dJointSetHingeParam(frWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHingeParam(frWheel.joint, dParamFMax, 1000.0);
    dJointSetHingeParam (frWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHingeParam (frWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHingeParam (frWheel.joint,dParamFudgeFactor,0.1);

    dJointSetHinge2Param(flWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(flWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (flWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamFudgeFactor,0.1);*/


    // speed
    if (brakeYes)
    {
        dReal factor= 0.1;
        dJointSetHingeParam(rrSuspension.jRotorMid, dParamVel
                            ,dJointGetHingeParam(rrSuspension.jRotorMid,dParamVel)*factor);
        dJointSetHingeParam(rrSuspension.jRotorMid, dParamFMax, 1000.0);
        dJointSetHingeParam(rlSuspension.jRotorMid, dParamVel
                            ,dJointGetHingeParam(rlSuspension.jRotorMid,dParamVel)*factor);
        dJointSetHingeParam(rlSuspension.jRotorMid, dParamFMax, 1000.0);

        dJointSetHingeParam(frSuspension.jRotorMid, dParamVel, realSpeed);//dJointGetHinge2Param(frWheel.joint,dParamVel2)*factor);
        dJointSetHingeParam(frSuspension.jRotorMid, dParamFMax, 1000.0);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamVel, realSpeed);//dJointGetHinge2Param(flWheel.joint,dParamVel2)*factor);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamFMax, 1000.0);
    }
    else
    {
        // turn off rear wheels
        dJointSetHingeParam(rrSuspension.jRotorMid, dParamFMax, 0.0);
        dJointSetHingeParam(rlSuspension.jRotorMid, dParamFMax, 0.0);
        // set up front wheels
        dJointSetHingeParam(frSuspension.jRotorMid, dParamVel, realSpeed);
        dJointSetHingeParam(frSuspension.jRotorMid, dParamFMax, 500.0);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamVel, realSpeed);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamFMax, 500.0);
        //printf("real speed: %f", realSpeed);
    }

    // reset manual mode flag
    manualYes= 0;

}

void vmWishboneCar::simForward(dReal mphSpeed, dReal targetStr)
{
    dReal dsteer, realSpeed;
    //dsteer = 0.0 - dJointGetHinge2Angle1(frWheel.joint);  // target at zero steer

    // calculate wheel real speed
    dReal siSpeed= mphSpeed*1.6/3600.0*1000; // convert ot m/s
    realSpeed = -siSpeed/frWheel.radius;

    // steer
    /*dJointSetHinge2Param(frWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(frWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (frWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamFudgeFactor,0.1);

    dJointSetHinge2Param(flWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(flWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (flWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamFudgeFactor,0.1);*/
    // speed
    if (brakeYes)
    {
        dReal factor= 0.1;
        dJointSetHingeParam(rrSuspension.jRotorMid, dParamVel, dJointGetHingeParam(rrSuspension.jRotorMid,dParamVel)*factor);
        dJointSetHingeParam(rrSuspension.jRotorMid, dParamFMax, 1000.0);
        dJointSetHingeParam(rlSuspension.jRotorMid, dParamVel, dJointGetHingeParam(rlSuspension.jRotorMid,dParamVel)*factor);
        dJointSetHingeParam(rlSuspension.jRotorMid, dParamFMax, 1000.0);
        dJointSetHingeParam(frSuspension.jRotorMid, dParamVel, realSpeed);//dJointGetHinge2Param(frSuspension.jRotorMid,dParamVel2)*factor);
        dJointSetHingeParam(frSuspension.jRotorMid, dParamFMax, 1000.0);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamVel, realSpeed);//dJointGetHinge2Param(flSuspension.jRotorMid,dParamVel2)*factor);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamFMax, 1000.0);
    }
    else
    {
        // turn off rear wheels
        dJointSetHingeParam(rrSuspension.jRotorMid, dParamFMax, 0.0);
        dJointSetHingeParam(rlSuspension.jRotorMid, dParamFMax, 0.0);
        // set up front wheels
        dJointSetHingeParam(frSuspension.jRotorMid, dParamVel, realSpeed);
        dJointSetHingeParam(frSuspension.jRotorMid, dParamFMax, 500.0);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamVel, realSpeed);
        dJointSetHingeParam(flSuspension.jRotorMid, dParamFMax, 500.0);
    }

}

void vmWishboneCar::simDraw()
{
    // draw suspension linkages
    drawLinkage(vm::FR);
    drawLinkage(vm::FL);
    drawLinkage(vm::RR);
    drawLinkage(vm::RL);

    // draw wheels
    dsSetColorAlpha(1.0,1.0,0.0,0.5);
    dsDrawCylinderD(dBodyGetPosition(frWheel.body),dBodyGetRotation(frWheel.body),frWheel.length,frWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(flWheel.body),dBodyGetRotation(frWheel.body),flWheel.length,flWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(rrWheel.body),dBodyGetRotation(rrWheel.body),rrWheel.length,rrWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(rlWheel.body),dBodyGetRotation(rlWheel.body),rlWheel.length,rlWheel.radius);

    // draw chassis
    dsSetColorAlpha(1.0,0.0,0.0,0.5); // red
    dsDrawBoxD(dBodyGetPosition(chassis.body),dBodyGetRotation(chassis.body)
               ,chassis.sides);


}

void vmWishboneCar::drawLinkage(vm::WheelLoc loc)
{
    // select wheel
	vmWishbone *snow = nullptr;
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

    // setup linkage
    const dReal *pos1,*R1,*pos2,*R2,*pos3,*R3;
    const dReal *pos4,*R4,*pos5,*R5;
    dsSetColorAlpha(0.0,0.0,1.0, 0.5);
    pos1 = dBodyGetPosition(snow->uplink.body);
    R1   = dBodyGetRotation(snow->uplink.body);
    dsDrawCylinderD(pos1,R1,snow->uplink.width,snow->uplink.radius);

    dsSetColor(0.0,1.0,0.0);
    pos2 = dBodyGetPosition(snow->hlink.body);
    R2   = dBodyGetRotation(snow->hlink.body);
    dsDrawCylinderD(pos2,R2,snow->hlink.width,snow->hlink.radius);

    dsSetColorAlpha(1.0,0.0,1.0, 0.5);
    pos3 = dBodyGetPosition(snow->lowlink.body);
    R3 = dBodyGetRotation(snow->lowlink.body);
    dsDrawCylinderD(pos3,R3,snow->lowlink.width,snow->lowlink.radius);

    // setup spring and damper
    dsSetColorAlpha(0.1,0.7,1.0, 0.5);
    pos4= dBodyGetPosition(snow->upstrut.body);
    R4= dBodyGetRotation(snow->upstrut.body);
    dsDrawCylinderD(pos4,R4,snow->upstrut.width,snow->upstrut.radius);

    dsSetColorAlpha(1.0,0.7,1.0, 0.5);
    pos5= dBodyGetPosition(snow->lowstrut.body);
    R5= dBodyGetRotation(snow->lowstrut.body);
    dsDrawCylinderD(pos5,R5,snow->lowstrut.width,snow->lowstrut.radius);


}

dReal vmWishboneCar::getSuspensionRate(vm::WheelLoc loc, dReal step)
{
    // select wheel
    vmWishbone *snow= nullptr;
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
    return dJointGetSliderPositionRate(snow->jStrutMid);
}

void vmWishboneCar::buildWheelJointLinkage(vmWheel *wnow, vmWishbone *snow, dReal shiftSign)
{
    // some parameters
    dMass m1;
    dMatrix3 rmat;
    const dReal *wheelPos= dBodyGetPosition(wnow->body);

    // uplink
    snow->uplink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->uplink.mass,3
                          ,snow->uplink.radius
                          ,snow->uplink.width);
    dBodySetMass(snow->uplink.body,&m1);
    dBodySetPosition(snow->uplink.body, wheelPos[0]
            ,wheelPos[1]+0.5*shiftSign*(snow->uplink.width)
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
            ,wheelPos[1]+0.5*shiftSign*(snow->lowlink.width)
            ,wheelPos[2]-0.5*snow->hlink.width);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 90*M_PI/180.0);
    dBodySetRotation(snow->lowlink.body,rmat);

    // lowstrut
    dReal y,z,angle;
    angle= -shiftSign*strutAngle;
    y= wheelPos[1]+0.1*shiftSign*(snow->lowlink.width)
            +0.5*shiftSign*snow->lowstrut.width*fabs(sin(angle));
    z= wheelPos[2]-0.5*snow->hlink.width+ 0.5*snow->lowstrut.width*cos(angle);

    snow->lowstrut.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->lowstrut.mass,3
                          ,snow->lowstrut.radius,snow->uplink.width);
    dBodySetMass(snow->lowstrut.body, &m1);
    dBodySetPosition(snow->lowstrut.body,wheelPos[0],y,z);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, angle);
    dBodySetRotation(snow->lowstrut.body,rmat);

    // upstrut
    snow->upstrut.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,snow->upstrut.mass,3
                          ,snow->upstrut.radius,snow->uplink.width);
    dBodySetMass(snow->upstrut.body, &m1);
    y= wheelPos[1]+0.1*shiftSign*(snow->lowlink.width)
            +shiftSign*snow->lowstrut.width*fabs(sin(angle))
            +0.5*shiftSign*snow->upstrut.width*fabs(sin(angle));
    z= wheelPos[2]-0.5*snow->hlink.width
            +1.0*snow->lowstrut.width*cos(angle)
            +0.5*snow->upstrut.width*cos(angle);
    dBodySetPosition(snow->upstrut.body,wheelPos[0],y,z);
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, angle);
    dBodySetRotation(snow->upstrut.body,rmat);



}

