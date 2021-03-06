#include "vmTruck.h"
#include <vector>

vmCar::vmTruck(dWorldID world, dSpaceID space)
{
    this->world= world;
    this->space= space;    
}


vmCar::~vmCar()
{
    //dtor
    delete frWheel.feedback;
    delete flWheel.feedback;
    delete rrWheel.feedback;
    delete rlWheel.feedback;
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

void vmCar::simForward(dReal mphSpeed, dReal targetSteer)
{
    dReal dsteer, realSpeed;
    dsteer = targetSteer - dJointGetHinge2Angle1(frWheel.joint);  // target at zero steer

    // calculate wheel real speed
    dReal siSpeed= mphSpeed*1.6/3600.0*1000; // convert ot m/s
    realSpeed = -siSpeed/frWheel.radius;
	
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

dReal vmCar::getNonlinearKd(vm::WheelLoc loc, dReal step)
{
    // select wheel
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

    // compute vz
    dReal vz= getSuspensionRate(loc, step);


    // calculate kd
    dReal kd;
    if (vz<0.0)
        kd= 1000.0;
    else if (vz<250.0)
        kd= 4000.0;
    else
        kd= 1539.0;

    //printf("dz=%12.6f; vz=%12.6f; kd=%12.6f\n",
    //       dzNew, vz, kd);
    return kd;
}

dReal vmCar::getSuspensionRate(vm::WheelLoc loc, dReal step)
{
    // select wheel
    vmWheel *wnow= nullptr;
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

    dVector3 res1,res2;
    dJointGetHinge2Anchor(wnow->joint, res1);
    dJointGetHinge2Anchor2(wnow->joint, res2);
    dReal dzNew = res1[2]- res2[2];
    // update old dz
    wnow->dzSuspension= dzNew;

    // compute vz
    return (dzNew- wnow->dzSuspension)/step;
}


void vmCar::listVehiclePosition(std::ofstream &fp, dReal simCt, dReal step)
{
    const dReal *pos= dBodyGetPosition(chassis.body);

    const dReal *ep= dBodyGetQuaternion(chassis.body);
    dReal roll= ep[0]*ep[1];
    dReal pitch= ep[0]*ep[2];
    dReal yaw= ep[0]*ep[3];

    if (simCt==1)
    {
        fp<<"Time,"<<"PosX,"<<"PosY,"<<"PosZ,"<<"Roll,"<<"Pitch,"<<"Yaw\n";        
    }
	fp << simCt*step << "," << pos[0] << "," << pos[1] << "," << pos[2]
		<< "," << roll << "," << pitch << "," << yaw << "\n";
    
}

void vmCar::listWheelForce(std::ofstream &fp, dReal simCt, dReal step)
{

    dJointFeedback *fb0= frWheel.feedback;
    dJointFeedback *fb1= flWheel.feedback;
    dJointFeedback *fb2= rrWheel.feedback;
    dJointFeedback *fb3= rlWheel.feedback;

	if (simCt == 1)
	{
		fp << "Time," << "FRChss-Fx," << "FRChss-Fy," << "FRChss-Fz," <<
			"FLChss-Fx," << "FLChss-Fy," << "FLChss-Fz," <<
			"RRChss-Fx," << "RRChss-Fy," << "RRChss-Fz," <<
			"RLChss-Fx," << "RLChss-Fy," << "RLChss-Fz\n";
	}
    fp<<simCt*step<<","<<fb0->f1[0]<<","<<fb0->f1[1]<<","<<fb0->f1[2]<<","<<
                fb1->f1[0]<<","<<fb1->f1[1]<<","<<fb1->f1[2]<<","<<
                fb2->f1[0]<<","<<fb2->f1[1]<<","<<fb2->f1[2]<<","<<
                fb3->f1[0]<<","<<fb3->f1[1]<<","<<fb3->f1[2]<<"\n";
    
    
}

void vmCar::setWheelJoint(vm::WheelLoc loc)
{
    vmWheel *wnow= nullptr;
    bool lock= false;
    dReal strutShift;

    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        strutShift= 0.2;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        strutShift= -0.2;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        lock= true;
        strutShift= 0.2;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        lock= true;
        strutShift= -0.2;
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

    // set joint feedback
    wnow->feedback= new dJointFeedback;
    dJointSetFeedback(wnow->joint,wnow->feedback);

}

void vmCar::setWheelSuspension(vm::WheelLoc loc, dReal step, dReal kps, dReal kds)
{
    vmWheel *wnow= nullptr;
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

void vmCar::simCommand(int cmd)
{
    // manual command
    manualYes= 1;
    brakeYes= 0;
    switch (cmd)
    {
    case 'i': case 'I':
        speed+= 0.25*M_PI;
        steer*= 0.98;
        break;
    case ',': case '<':
        speed*= 0.1*M_PI;
        brakeYes = 1;
        steer*= 0.98;
        break;
    case 'k': case 'K':
        speed-= 0.25*M_PI;
        steer*= 0.98;
        break;
    case 'j': case 'J':
        steer= -40*M_PI/180;
        break;
    case 'l': case 'L':
        steer= 40*M_PI/180;
        break;
    default:
        manualYes = 0;
        steer*= 0.98;
        speed*= 0.999;
    }

    //setInitialControls(steer,speed,steerGain);
}

void vmCar::setWheel(vm::WheelLoc loc, dReal mass, dReal length, dReal radius)
{
    vmWheel *wnow= nullptr;
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




