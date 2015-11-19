#include "vmCar.h"

vmCar::vmCar(dWorldID world, dSpaceID space)
{
    this->world= world;
    this->space= space;
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

    // create body and geom
    chassis.body= dBodyCreate(world);
    chassis.geom= dCreateBox(space,chassis.sides[0],chassis.sides[1],chassis.sides[2]);

    // mass properties
    dMass m1;
    dMassSetZero(&m1);
    dMassSetBoxTotal(&m1,chassis.mass,
                     chassis.sides[0],chassis.sides[1],chassis.sides[2]);
    dBodySetMass(chassis.body,&m1);
    dGeomSetBody(chassis.geom,chassis.body);
}

void vmCar::setCMPosition(dReal x, dReal y, dReal z)
{
    // if any of chassis and wheels are not setup, return
    if ((chassis.body==0) || (frWheel.body==0) ||
            (flWheel.body==0) || (rrWheel.body==0) ||
            (rlWheel.body==0))
    {
        printf('Some chassis/wheel entity is not defined yet!');
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

void vmCar::setAllWheelJoint(dReal kps, dReal kds)
{
    // if any of chassis and wheels are not setup, return
    if ((chassis.body==0) || (frWheel.body==0) ||
            (flWheel.body==0) || (rrWheel.body==0) ||
            (rlWheel.body==0))
    {
        printf('Some chassis/wheel entity is not defined yet!');
        return;
    }

    setWheelJoint(vmCar::WheelLoc::FR,kps,kds);
    setWheelJoint(vmCar::WheelLoc::FL,kps,kds);
    setWheelJoint(vmCar::WheelLoc::RR,kps,kds);
    setWheelJoint(vmCar::WheelLoc::RL,kps,kds);

}

void vmCar::setWheelJoint(vmCar::WheelLoc loc,dReal kps, dReal kds)
{
    vmWheel *wnow;
    bool lock= false;
    switch (loc) {
    case vmCar::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vmCar::WheelLoc::FL:
        wnow= &frWheel;
        break;
    case vmCar::WheelLoc::RR:
        wnow= &frWheel;
        lock= true;
        break;
    case vmCar::WheelLoc::RL:
        wnow= &frWheel;
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

    dJointSetHinge2Param(wnow->joint,dParamSuspensionERP, computeERP(kps,kds)); // suspension
    dJointSetHinge2Param(wnow->joint,dParamSuspensionCFM, computeCFM(kps,kds)); // suspension

    // lock rear wheel to keep it align to 0 degree (longitudinally)
    if (lock)
    {
        dJointSetHinge2Param(wnow->joint, dParamLoStop, 0.0);
        dJointSetHinge2Param(wnow->joint, dParamHiStop, 0.0);
    }
}

void vmCar::setWheel(vmCar::WheelLoc loc, dReal mass, dReal length, dReal radius)
{
    vmWheel *wnow;
    switch (loc) {
    case vmCar::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vmCar::WheelLoc::FL:
        wnow= &frWheel;
        break;
    case vmCar::WheelLoc::RR:
        wnow= &frWheel; break;
    case vmCar::WheelLoc::RL:
        wnow= &frWheel;
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
}

void vmCar::setAllWheel(dReal mass, dReal length, dReal radius)
{
    setWheel(vmCar::WheelLoc::FR,mass,length,radius);
    setWheel(vmCar::WheelLoc::FL,mass,length,radius);
    setWheel(vmCar::WheelLoc::RR,mass,length,radius);
    setWheel(vmCar::WheelLoc::RL,mass,length,radius);
}


// compute ERP and CFM
dReal vmCar::computeERP(dReal kp, dReal kd)
{
    return STEPSIZE*kp/(STEPSIZE*kp + kd);
}
dReal vmCar::computeCFM(dReal kp, dReal kd)
{
    return 1.0/(STEPSIZE*kp + kd);
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


