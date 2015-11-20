#include "vmCar.h"

vmCar::vmCar(dWorldID world, dSpaceID space)
{
    this->world= world;
    this->space= space;
    this->brakeYes=0;
    this->manualYes=0;
}


vmCar::~vmCar()
{
    //dtor
}

/* set chassis dimension length, width, and height */
void vmCar::setChassis(dReal mass, dReal length, dReal width, dReal height)
{
    chassis.sides[0]= length;
    chassis.sides[1]= width;
    chassis.sides[2]= height;
    chassis.mass= mass;

    // create body and getom
    chassis.body= dBodyCreate(world);
    chassis.geom= dCreateBox(space,chassis.sides[0],chassis.sides[1],chassis.sides[2]);

    // mass properties
    dMass m1;
    dMassSetZero(&m1);
    dMassSetBoxTotal(&m1,chassis.mass,
                     chassis.sides[0],chassis.sides[1],chassis.sides[2]);
    dBodySetMass(chassis.body,&m1);
    dGeomSetBody(chassis.geom,chassis.body);

    // set default controls
    this->steer= 0.0;
    this->speed= 0.0;
    this->steerGain= 100;

    chassis.initialized=true;
}

void vmCar::setCMPosition(dReal x, dReal y, dReal z)
{
    // if any of chassis and wheels are not setup, return
    if ((!chassis.initialized) || (!frWheel.initialized) ||
            (!flWheel.initialized) || (!rrWheel.initialized) ||
            (!rlWheel.initialized))
    {
        printf("setCM: Some chassis/wheel entity is not defined yet!");
        return;
    }

    // set chassis position
    dBodySetPosition(chassis.body,x,y,z);

    // set locations
    dReal len= chassis.sides[0];
    dReal wd= chassis.sides[1];
    dReal ht= chassis.sides[2];

    dBodySetPosition(frWheel.body,0.5*len, -0.5*wd, z-0.5*ht);
    dBodySetPosition(flWheel.body,0.5*len, 0.5*wd, z-0.5*ht);
    dBodySetPosition(rrWheel.body,-0.5*len, -0.5*wd, z-0.5*ht);
    dBodySetPosition(rlWheel.body,-0.5*len, 0.5*wd, z-0.5*ht);

    //set rotations
    dMatrix3 rmat;
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 0.5*M_PI);
    dBodySetRotation(frWheel.body, rmat);
    dBodySetRotation(flWheel.body, rmat);
    dBodySetRotation(rrWheel.body, rmat);
    dBodySetRotation(rlWheel.body, rmat);

}

/* place the car on the ground based on the x and y location
    z location will be automatically determined
*/
void vmCar::setCarOnGround(dReal x, dReal y)
{
    dReal znow;
    znow= frWheel.radius + 0.5*chassis.sides[2];
    setCMPosition(x,y,znow);
}

void vmCar::setAllWheelJoint()
{
    // if any of chassis and wheels are not setup, return
    if ((!chassis.initialized) || (!frWheel.initialized) ||
            (!flWheel.initialized) || (!rrWheel.initialized) ||
            (!rlWheel.initialized))
    {
        printf("set Joint: Some chassis/wheel entity is not defined yet!");
        return;
    }

    setWheelJoint(vm::WheelLoc::FR);
    setWheelJoint(vm::WheelLoc::FL);
    setWheelJoint(vm::WheelLoc::RR);
    setWheelJoint(vm::WheelLoc::RL);

}

/* simloop control subfunction
    inputs are control mode flags
    int manualYes: 1: under manual input; 0 otherwise
    int brakeYes: 1: brake activated; 0 otherwise
*/
void vmCar::simControl()
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
    dsteer = bounded(steer,-40.0,40.0) - dJointGetHinge2Angle1(frWheel.joint);
    realSpeed = -bounded(speed, -14*M_PI, 30*M_PI);
    // steer
    dJointSetHinge2Param(frWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(frWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (frWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamFudgeFactor,0.1);

    dJointSetHinge2Param(flWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(flWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (flWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamFudgeFactor,0.1);
    // speed
    if (brakeYes)
    {
        dReal factor= 0.1;
        dJointSetHinge2Param(rrWheel.joint, dParamVel2, dJointGetHinge2Param(rrWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(rrWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(rlWheel.joint, dParamVel2, dJointGetHinge2Param(rlWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(rlWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(frWheel.joint, dParamVel2, realSpeed);//dJointGetHinge2Param(frWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(frWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(flWheel.joint, dParamVel2, realSpeed);//dJointGetHinge2Param(flWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(flWheel.joint, dParamFMax2, 1000.0);
    }
    else
    {
        // turn off rear wheels
        dJointSetHinge2Param(rrWheel.joint, dParamFMax2, 0.0);
        dJointSetHinge2Param(rlWheel.joint, dParamFMax2, 0.0);
        // set up front wheels
        dJointSetHinge2Param(frWheel.joint, dParamVel2, realSpeed);
        dJointSetHinge2Param(frWheel.joint, dParamFMax2, 500.0);
        dJointSetHinge2Param(flWheel.joint, dParamVel2, realSpeed);
        dJointSetHinge2Param(flWheel.joint, dParamFMax2, 500.0);
    }

    // reset manual mode flag
    manualYes= 0;

}

void vmCar::simDraw()
{
    // draw chassis
    dsSetColor(1.0,0.0,0.0); // red
    dsDrawBoxD(dBodyGetPosition(chassis.body),dBodyGetRotation(chassis.body)
               ,chassis.sides);
    // draw wheels
    dsSetColor(1.0,1.0,0.0);
    dsDrawCylinderD(dBodyGetPosition(frWheel.body),dBodyGetRotation(frWheel.body),frWheel.length,frWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(flWheel.body),dBodyGetRotation(frWheel.body),flWheel.length,flWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(rrWheel.body),dBodyGetRotation(rrWheel.body),rrWheel.length,rrWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(rlWheel.body),dBodyGetRotation(rlWheel.body),rlWheel.length,rlWheel.radius);

}

void vmCar::setInitialControls(dReal steer, dReal speed, dReal steerGain)
{
    this->steer= steer;
    this->speed= speed;
    this->steerGain= steerGain;
}

dReal vmCar::getTotalMass()
{
    dReal mass= chassis.mass + frWheel.mass + flWheel.mass
            + rrWheel.mass + rlWheel.mass;
    return mass;
}

void vmCar::setWheelJoint(vm::WheelLoc loc)
{
    vmWheel *wnow;
    bool lock= false;
    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        lock= true;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        lock= true;
        break;
    default:
        break;
    }

    // create joint
    wnow->joint= dJointCreateHinge2(world,0);
    // attach joint
    dJointAttach(wnow->joint,chassis.body,wnow->body);
    // set joint
    const dReal *pos;
    pos= dBodyGetPosition(wnow->body);
    dJointSetHinge2Anchor(wnow->joint,pos[0],pos[1],pos[2]);
    dJointSetHinge2Axis1(wnow->joint,0.0, 0.0, 1.0); // steering
    dJointSetHinge2Axis2(wnow->joint,0.0, 1.0, 0.0); // rotation

    // lock rear wheel to keep it align to 0 degree (longitudinally)
    if (lock)
    {
        dJointSetHinge2Param(wnow->joint, dParamLoStop, 0.0);
        dJointSetHinge2Param(wnow->joint, dParamHiStop, 0.0);
    }
}

void vmCar::setWheelSuspension(vm::WheelLoc loc, dReal step, dReal kps, dReal kds)
{
    vmWheel *wnow;
    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        break;
    default:
        break;
    }

    dJointSetHinge2Param(wnow->joint,dParamSuspensionERP, computeERP(step,kps,kds)); // suspension
    dJointSetHinge2Param(wnow->joint,dParamSuspensionCFM, computeCFM(step,kps,kds)); // suspension
}

void vmCar::setAllWheelSuspension(dReal step, dReal kps, dReal kds)
{
    setWheelSuspension(vm::WheelLoc::FR,step,kps,kds);
    setWheelSuspension(vm::WheelLoc::FL,step,kps,kds);
    setWheelSuspension(vm::WheelLoc::RR,step,kps,kds);
    setWheelSuspension(vm::WheelLoc::RL,step,kps,kds);
}

void vmCar::setWheel(vm::WheelLoc loc, dReal mass, dReal length, dReal radius)
{
    vmWheel *wnow;
    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        break;
    default:
        break;
    }

    // setup now
    wnow->radius= radius;
    wnow->length= length;
    wnow->mass= mass;

    wnow->body= dBodyCreate(world);
    wnow->geom= dCreateCylinder(space,wnow->radius,wnow->length);

    // set mass (initially axial direction is parallel to global z)
    dMass m1;
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,wnow->mass,3,wnow->radius,wnow->length);
    dBodySetMass(wnow->body,&m1);

    // set geom
    dGeomSetBody(wnow->geom,wnow->body);

    wnow->initialized= true;
}

void vmCar::setAllWheel(dReal mass, dReal length, dReal radius)
{
    setWheel(vm::WheelLoc::FR,mass,length,radius);
    setWheel(vm::WheelLoc::FL,mass,length,radius);
    setWheel(vm::WheelLoc::RR,mass,length,radius);
    setWheel(vm::WheelLoc::RL,mass,length,radius);
}


// compute ERP and CFM
dReal vmCar::computeERP(dReal step, dReal kp, dReal kd)
{
    return step*kp/(step*kp + kd);
}
dReal vmCar::computeCFM(dReal step, dReal kp, dReal kd)
{
    return 1.0/(step*kp + kd);
}

//bounded function
dReal vmCar::bounded(dReal var, dReal lb, dReal ub)
{
    dReal out;
    if (var<lb)
        out= lb;
    else if (var>ub)
        out= ub;
    else
        out= var;
    return out;
}


