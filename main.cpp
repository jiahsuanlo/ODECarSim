// Simple ODE car simulator


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <include/vmCar.h>


#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere dsDrawSphereD
#endif

static dWorldID world;
static dSpaceID space;
static dSpaceID groundSpace;
static dJointGroupID contactGroup;

// define body type
vmCar car(world,space);
dsFunctions fn;

// define some constants
dReal STEPSIZE= 0.01;

// obstacle balls
const int nObs= 250;
sphere ball[nObs];

// chassis parameters
dReal chassisMass= 3000;
dReal chassisLength= 2.0;
dReal chassisWidth= 1.5;
dReal chassisHeight= 0.75;

// wheel paremeters
dReal wheelMass= 10;
dReal wheelRadius= 0.3;  // wheel radius
dReal wheelWidth= 0.2;  // wheel width
dReal kps= 10000.0;  // suspension stiffness
dReal kds= 1000.0;   // suspension damping

// control parameters
dReal speed= 0.0;
dReal steer= 0.0;
dReal steerGain= 100;

// control mode flags
int manualYes= 0;
int brakeYes=0;


// collision callback
static void nearCallback(void *data, dGeomID g1, dGeomID g2)
{
    static int maxContacts= 10;

    // check contacts
    dContact contact[maxContacts];
    int nContact= dCollide(g1,g2,maxContacts,&contact[0].geom,sizeof(dContact));

    // treat contacts
    if (nContact>0)
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

// simulation loop
static void simloop(int pause)
{

    car.simControl(manualYes,brakeYes);

    dSpaceCollide2(space,groundSpace,0,&nearCallback);
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
    brakeYes= 0;
    switch (cmd)
    {
        case 'i': case 'I':
            speed+= 0.25*M_PI;
            break;
        case ',': case '<':
            speed*= 0.1*M_PI;
            brakeYes = 1;
            break;
        case 'k': case 'K':
            speed-= 0.25*M_PI;
            break;
        case 'j': case 'J':
            steer-= 5*M_PI/180;
            break;
        case 'l': case 'L':
            steer+= 5*M_PI/180;
            break;
        default:
            manualYes = 0;
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
    //fn.path_to_textures= "c:/dev/ode-0.13/drawstuff/textures";  // win version
    fn.path_to_textures= "/home/jlo/ode-0.13/drawstuff/textures";  // linux version
}

// create bodies
void createCar()
{
    car.setChassis(chassisMass,chassisLength,
                   chassisWidth,chassisHeight);
    car.setAllWheel(wheelMass,wheelWidth,wheelRadius);
    car.setCarOnGround(0.0,0.0);
    car.setAllWheelJoint(kps,kds);
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
        ball[i].geom= dCreateSphere(groundSpace, ball[i].radius);
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
    groundSpace= dHashSpaceCreate(0);
    contactGroup= dJointGroupCreate(0);

    //set gravity
    dWorldSetGravity(world,0.0,0.0, -9.81);

    // create ground
    ground= dCreatePlane(groundSpace,0.0, 0.0, 1.0, 0.0);  // z= 0 plane
    // create obstacles
    createObstacles();

    // create car
    createCar();

    // start simulation
    dsSimulationLoop(argc,argv,1000,800,&fn);

    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
