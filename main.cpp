// Simple ODE car simulator


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <include/vmCar.h>


#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere dsDrawSphereD
#endif

#ifdef _WIN32
   #define texturePath "c:/dev/ode-0.13/drawstuff/textures";  // win32 version
#elif __linux__
   #define texturePath "/home/jlo/ode-0.13/drawstuff/textures";  // linux version
#endif

static dWorldID world;
static dSpaceID space;
static dSpaceID vehicleSpace;
static dSpaceID groundSpace;
static dJointGroupID contactGroup;

// define body type
vmCar *car;
dsFunctions fn;

// define some constants
dReal STEPSIZE= 0.01;

// ground and obstacle balls
dGeomID ground;
const int nObs= 250;
sphere ball[nObs];

// chassis parameters
dReal chassisMass= 1000;
dReal chassisLength= 2.0;
dReal chassisWidth= 1.5;
dReal chassisHeight= 0.75;

// wheel paremeters
dReal wheelMass= 10;
dReal wheelRadius= 0.3;  // wheel radius
dReal wheelWidth= 0.2;  // wheel width
dReal kps= 100000.0;  // suspension stiffness
dReal kds= 8000.0;   // suspension damping

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

    if(dGeomIsSpace(g1) && dGeomIsSpace(g2))
    {
        // colliding a space with something
        dSpaceCollide2(g1,g2,data,&nearCallback);
    }
    else
    {
        // check contacts
        dContact contact[maxContacts];
        int nContact= dCollide(g1,g2,maxContacts,&contact[0].geom,sizeof(dContact));

        // treat contacts
        if (nContact>0)
        {
            for (int i=0; i<nContact; ++i)
            {
                contact[i].surface.mode = dContactApprox1_1|dContactApprox1_2
                        |dContactSoftCFM|dContactSoftERP;

                contact[i].surface.mu = 1;
                contact[i].surface.mu2 = 1;
                //contact[i].surface.slip1 = 1e-10;
                //contact[i].surface.slip2 = 1e-10;
                contact[i].surface.soft_erp = 0.01;
                contact[i].surface.soft_cfm = 1e-5;

                // build contact joint now
                dJointID c= dJointCreateContact(world,contactGroup,&contact[i]);
                dJointAttach(c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
                //dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));

            }
        }
    }
}

// simulation loop
static void simloop(int pause)
{

    // update suspension
    car->setAllWheelSuspension(STEPSIZE,kps,kds);
    //set control
    car->simControl();

    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,STEPSIZE);
    // collision setup
    dJointGroupEmpty(contactGroup);

    // draw car now
    car->simDraw();

    // draw obstacles
    for (int i=0;i<nObs;++i)
    {
        dsDrawSphere(dGeomGetPosition(ball[i].geom),dGeomGetRotation(ball[i].geom),ball[i].radius);
    }
}


// manual command
void manualCmd(int cmd)
{
    car->manualYes= 1;
    car->brakeYes= 0;
    switch (cmd)
    {
        case 'i': case 'I':
           car->speed+= 0.25*M_PI;
           car->steer*= 0.98;
            break;
        case ',': case '<':
            car->speed*= 0.1*M_PI;
            car->brakeYes = 1;
            car->steer*= 0.98;
            break;
        case 'k': case 'K':
            car->speed-= 0.25*M_PI;
            car->steer*= 0.98;
            break;
        case 'j': case 'J':
            car->steer= -40*M_PI/180;
            break;
        case 'l': case 'L':
            car->steer= 40*M_PI/180;
            break;
        default:
            car->manualYes = 0;
            car->steer*= 0.98;
            car->speed*= 0.999;
    }

    //car->setInitialControls(steer,speed,steerGain);
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
    fn.path_to_textures= texturePath;
    //fn.path_to_textures= "c:/dev/ode-0.13/drawstuff/textures";  // win version
    //fn.path_to_textures= "/home/jlo/ode-0.13/drawstuff/textures";  // linux version
}

// create bodies
void createCar()
{
    car= new vmCar(world,vehicleSpace);
    car->setChassis(chassisMass,chassisLength,
                   chassisWidth,chassisHeight);
    car->setAllWheel(wheelMass,wheelWidth,wheelRadius);
    car->setCarOnGround(0.0,0.0);
    car->setAllWheelJoint();
    car->setInitialControls(steer,speed,steerGain);
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
    vehicleSpace= dHashSpaceCreate(space);
    groundSpace= dHashSpaceCreate(space);
    dSpaceSetSublevel(groundSpace,1);
    dSpaceSetSublevel(vehicleSpace,1);

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
    dSpaceDestroy(space);
    dCloseODE();

    delete car;

    return 0;
}
