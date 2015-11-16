// Sample1.cpp by Kosei Demura 2005-2011
// My web site is http://demura.net
// This program uses the Open Dynamics Engine (ODE) by Russell Smith.
// The ODE web site is http://ode.org/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere dsDrawSphereD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactGroup;

// define body type
typedef struct{
    dBodyID body;
    dGeomID geom;
    dReal mass;
    dReal sides[3];  //length,width,height
} chassis;

typedef struct {
    dBodyID body;
    dGeomID geom;
    dReal mass;
    dReal length,radius;
} wheel;

typedef struct{
    dGeomID geom;
    dReal radius;
} sphere;


chassis carBody;
wheel FRWheel, FLWheel, RRWheel, RLWheel;
dJointID FRJoint, FLJoint, RRJoint, RLJoint;
dJointID FRMotor1, FRMotor2, FLMotor1, FLMotor2;

dsFunctions fn;

// define some constants
dReal STEPSIZE= 0.01;
dReal LN= 2.0;
dReal WD= 1.5;
dReal HT= 0.75;
dReal RD= 0.3;  // wheel radius
dReal WW= 0.2;  // wheel width

dReal speed= 0.0;
dReal steer= 0.0;

dReal kps= 10000.0;  // suspension stiffness
dReal kds= 1000.0;   // suspension damping

dReal steerGain= 100;
int manualYes= 0;

// obstacle balls
const int nObs= 250;
sphere ball[nObs];



// compute ERP and CFM
dReal computeERP(dReal kp, dReal kd)
{
    return STEPSIZE*kp/(STEPSIZE*kp + kd);
}
dReal computeCFM(dReal kp, dReal kd)
{
    return 1.0/(STEPSIZE*kp + kd);
}


// collision callback
static void nearCallback(void *data, dGeomID g1, dGeomID g2)
{
    static int maxContacts= 10;
    // exit if two body are connected
    dBodyID b1= dGeomGetBody(g1);
    dBodyID b2= dGeomGetBody(g2);
    if (b1 && b2 && dAreConnected(b1,b2))
        return;

    int isGround = ((ground == g1) | (ground == g2));

    for (int i=0;i<nObs;++i)
    {
        int isObstacle= ((ball[i].geom==g1) | (ball[i].geom==g2));
        isGround = isGround | isObstacle;
    }

    // check contacts
    dContact contact[maxContacts];
    int nContact= dCollide(g1,g2,maxContacts,&contact[0].geom,sizeof(dContact));

    // treat contacts
    if (isGround && nContact>0)
    {
        for (int i=0; i<nContact; ++i)
        {
            contact[i].surface.mode = dContactApprox1;
            contact[i].surface.mu = 1.0; //dInfinity;
            //contact[i].surface.slip1 = 0.001;
            //contact[i].surface.slip2 = 0.001;
            //contact[i].surface.soft_erp = 0.8;
            //contact[i].surface.soft_cfm = 1e-6;

            // build contact joint now
            dJointID c= dJointCreateContact(world,contactGroup,&contact[i]);
            dJointAttach(c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
            //dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));

        }
    }
}

dReal bounded(dReal var, dReal lb, dReal ub)
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

void simControl()
{

    /*if (manualYes)
        steerGain= 100;
    else
    {
        steerGain= 100;
        steer*= 0.8;
        //speed*= 0.8;
    }*/
    dReal dsteer, realSpeed;
    dsteer = bounded(steer,-40.0,40.0) - dJointGetHinge2Angle1(FRJoint);
    realSpeed = -bounded(speed, -14*M_PI, 30*M_PI);
    // steer
    dJointSetHinge2Param(FRJoint, dParamVel, steerGain*dsteer);
    dJointSetHinge2Param(FRJoint, dParamFMax, 1000.0);
    dJointSetHinge2Param (FRJoint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (FRJoint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (FRJoint,dParamFudgeFactor,0.1);

    dJointSetHinge2Param(FLJoint, dParamVel, steerGain*dsteer);
    dJointSetHinge2Param(FLJoint, dParamFMax, 1000.0);
    dJointSetHinge2Param (FLJoint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (FLJoint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (FLJoint,dParamFudgeFactor,0.1);
    // speed
    dJointSetHinge2Param(FRJoint, dParamVel2, realSpeed);
    dJointSetHinge2Param(FRJoint, dParamFMax2, 10000.0);
    dJointSetHinge2Param(FLJoint, dParamVel2, realSpeed);
    dJointSetHinge2Param(FLJoint, dParamFMax2, 10000.0);

    // reset manual mode flag
    manualYes= 0;
}

// simulation loop
static void simloop(int pause)
{

    simControl();
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,STEPSIZE);
    // collision setup
    dJointGroupEmpty(contactGroup);

    // draw chassis
    dsSetColor(1.0,0.0,0.0); // red
    dsDrawBoxD(dBodyGetPosition(carBody.body),dBodyGetRotation(carBody.body),carBody.sides);
    // draw wheels
    dsSetColor(1.0,1.0,0.0);
    dsDrawCylinderD(dBodyGetPosition(FRWheel.body),dBodyGetRotation(FRWheel.body),FRWheel.length,FRWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(FLWheel.body),dBodyGetRotation(FRWheel.body),FLWheel.length,FRWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(RRWheel.body),dBodyGetRotation(RRWheel.body),FRWheel.length,FRWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(RLWheel.body),dBodyGetRotation(RLWheel.body),FRWheel.length,FRWheel.radius);

    for (int i=0;i<nObs;++i)
    {
        dsDrawSphere(dGeomGetPosition(ball[i].geom),dGeomGetRotation(ball[i].geom),ball[i].radius);
    }
}




// manual command
void manualCmd(int cmd)
{
    manualYes= 1;
    switch (cmd)
    {
        case 'i': case 'I':
            speed+= 0.25*M_PI;
            break;
        case 'k': case 'K':
            speed-= 0.25*M_PI;
            break;
        case 'j': case 'J':
            steer-= 2*M_PI/180;
            break;
        case 'l': case 'L':
            steer+= 2*M_PI/180;
            break;
    }

}

// camera setup
void start()
{
  static float xyz[3] = {0.0,-30.0,10.0};
  static float hpr[3] = {90.0,-30.0,0.0};
  dsSetViewpoint (xyz,hpr);
}

// drawstuff initialization
void setDSFunctions()
{
    fn.version= DS_VERSION;
    fn.step= &simloop;
    fn.start= &start;
    fn.stop= NULL;
    fn.command= &manualCmd;
    fn.path_to_textures= "/home/jlo/ode-0.13/drawstuff/textures";
}

// create bodies
void createChassis()
{
    carBody.sides[0]= LN; // length
    carBody.sides[1]= WD; // width
    carBody.sides[2]= HT; // height
    carBody.mass= 1000;  // kg

    carBody.body= dBodyCreate(world);
    carBody.geom= dCreateBox(space,carBody.sides[0],carBody.sides[1],carBody.sides[2]);

    // mass properties
    dMass m1;
    dMassSetZero(&m1);
    dMassSetBoxTotal(&m1,carBody.mass,
                     carBody.sides[0],carBody.sides[1],carBody.sides[2]);
    dBodySetMass(carBody.body,&m1);
    dBodySetPosition(carBody.body,0.0,0.0,RD+0.5*HT);
    dGeomSetBody(carBody.geom,carBody.body);
}
// create wheels
void createWheels()
{
    FRWheel.radius= RD; FRWheel.length=WW; FRWheel.mass= 8;
    FLWheel.radius= RD; FLWheel.length=WW; FLWheel.mass= 8;
    RRWheel.radius= RD; RRWheel.length=WW; RRWheel.mass= 8;
    RLWheel.radius= RD; RLWheel.length=WW; RLWheel.mass= 8;

    // create body and geom
    FRWheel.body= dBodyCreate(world);
    FRWheel.geom= dCreateCylinder(space,FRWheel.radius,FRWheel.length);
    FLWheel.body= dBodyCreate(world);
    FLWheel.geom= dCreateCylinder(space,FLWheel.radius,FLWheel.length);
    RRWheel.body= dBodyCreate(world);
    RRWheel.geom= dCreateCylinder(space,RRWheel.radius,RRWheel.length);
    RLWheel.body= dBodyCreate(world);
    RLWheel.geom= dCreateCylinder(space,RLWheel.radius,RLWheel.length);

    // set mass
    dMass m1;
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,FRWheel.mass,2,FRWheel.radius,FRWheel.length);
    dBodySetMass(FRWheel.body,&m1);

    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,FLWheel.mass,2,FLWheel.radius,FLWheel.length);
    dBodySetMass(FLWheel.body,&m1);

    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,RRWheel.mass,2,RRWheel.radius,RRWheel.length);
    dBodySetMass(RRWheel.body,&m1);

    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,RLWheel.mass,2,RLWheel.radius,RLWheel.length);
    dBodySetMass(RLWheel.body,&m1);

    // set locations
    dBodySetPosition(FRWheel.body,0.5*LN, 0.5*WD, RD);
    dBodySetPosition(FLWheel.body,0.5*LN, -0.5*WD, RD);
    dBodySetPosition(RRWheel.body,-0.5*LN, 0.5*WD, RD);
    dBodySetPosition(RLWheel.body,-0.5*LN, -0.5*WD, RD);

    //set rotations
    dMatrix3 rmat;
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0, 0.5*M_PI);
    dBodySetRotation(FRWheel.body, rmat);
    dBodySetRotation(FLWheel.body, rmat);
    dBodySetRotation(RRWheel.body, rmat);
    dBodySetRotation(RLWheel.body, rmat);

    // set geom
    dGeomSetBody(FRWheel.geom,FRWheel.body);
    dGeomSetBody(FLWheel.geom,FLWheel.body);
    dGeomSetBody(RRWheel.geom,RRWheel.body);
    dGeomSetBody(RLWheel.geom,RLWheel.body);
}

// create wheel-body joints
void createJoints()
{
    // create joint
    FRJoint= dJointCreateHinge2(world,0);
    FLJoint= dJointCreateHinge2(world,0);
    RRJoint= dJointCreateHinge2(world,0);
    RLJoint= dJointCreateHinge2(world,0);

    // attach joint
    dJointAttach(FRJoint,carBody.body,FRWheel.body);
    dJointAttach(FLJoint,carBody.body,FLWheel.body);
    dJointAttach(RRJoint,carBody.body,RRWheel.body);
    dJointAttach(RLJoint,carBody.body,RLWheel.body);

    // set joint
    const dReal *pos;
    pos= dBodyGetPosition(FRWheel.body);
    dJointSetHinge2Anchor(FRJoint,pos[0],pos[1],pos[2]);
    dJointSetHinge2Axis1(FRJoint,0.0, 0.0, 1.0); // steering
    dJointSetHinge2Axis2(FRJoint,0.0, 1.0, 0.0); // rotation
    dJointSetHinge2Param(FRJoint,dParamSuspensionERP, computeERP(kps,kds)); // suspension
    dJointSetHinge2Param(FRJoint,dParamSuspensionCFM, computeCFM(kps,kds)); // suspension

    pos= dBodyGetPosition(FLWheel.body);
    dJointSetHinge2Anchor(FLJoint,pos[0],pos[1],pos[2]);
    dJointSetHinge2Axis1(FLJoint,0.0, 0.0, 1.0); // steering
    dJointSetHinge2Axis2(FLJoint,0.0, 1.0, 0.0); // rotation
    dJointSetHinge2Param(FLJoint,dParamSuspensionERP, computeERP(kps,kds)); // suspension
    dJointSetHinge2Param(FLJoint,dParamSuspensionCFM, computeCFM(kps,kds)); // suspension

    pos= dBodyGetPosition(RRWheel.body);
    dJointSetHinge2Anchor(RRJoint,pos[0],pos[1],pos[2]);
    dJointSetHinge2Axis1(RRJoint,0.0, 0.0, 1.0); // steering
    dJointSetHinge2Axis2(RRJoint,0.0, 1.0, 0.0); // rotation
    dJointSetHinge2Param(RRJoint,dParamSuspensionERP, computeERP(kps,kds)); // suspension
    dJointSetHinge2Param(RRJoint,dParamSuspensionCFM, computeCFM(kps,kds)); // suspension

    pos= dBodyGetPosition(RLWheel.body);
    dJointSetHinge2Anchor(RLJoint,pos[0],pos[1],pos[2]);
    dJointSetHinge2Axis1(RLJoint,0.0, 0.0, 1.0); // steering
    dJointSetHinge2Axis2(RLJoint,0.0, 1.0, 0.0); // rotation
    dJointSetHinge2Param(RLJoint,dParamSuspensionERP, computeERP(kps,kds)); // suspension
    dJointSetHinge2Param(RLJoint,dParamSuspensionCFM, computeCFM(kps,kds)); // suspension

    // lock rear wheel to keep it align to 0 degree (longitudinally)
    dJointSetHinge2Param(RRJoint, dParamLoStop, 0.0);
    dJointSetHinge2Param(RRJoint, dParamHiStop, 0.0);
    dJointSetHinge2Param(RLJoint, dParamLoStop, 0.0);
    dJointSetHinge2Param(RLJoint, dParamHiStop, 0.0);

}

// create obstacles
void createObstacles()
{
    dReal xo, yo;

    for (int i=0; i<nObs; ++i)
    {
        xo= dRandReal()*50;
        yo= dRandReal()*50;
        ball[i].radius= 1.0;
        ball[i].geom= dCreateSphere(space, ball[i].radius);
        dGeomSetPosition(ball[i].geom,xo,-yo,-0.9);
    }
}

// main
int main (int argc, char **argv)
{
    setDSFunctions();
    dInitODE();
    // create basis
    world= dWorldCreate();
    space= dHashSpaceCreate(0);
    contactGroup= dJointGroupCreate(0);

    //set gravity
    dWorldSetGravity(world,0.0,0.0, -9.81);

    // create ground
    ground= dCreatePlane(space,0.0, 0.0, 1.0, 0.0);  // z= 0 plane

    // create bodies
    createChassis();
    createWheels();

    // create wheel-body joints
    createJoints();

    // create obstacles
    createObstacles();

    // start simulation
    dsSimulationLoop(argc,argv,1000,800,&fn);

    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
