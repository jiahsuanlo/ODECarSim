// Simple suspension test


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

static dWorldID world;
static dSpaceID space;
static dSpaceID carSpace;
static dSpaceID groundSpace;
static dGeomID ground;
static dJointGroup contactGroup;
dJointFeedback *cgFeedback= new dJointFeedback;

struct cylinder{
    dBodyID body;
    dGeomID geom;
    dReal radius;
    dReal width;
    dReal mass;
    cylinder(dReal mass,dReal radius, dReal width):
        mass(mass), radius(radius), width(width){}
};

struct box{
    dBodyID body;
    dGeomID geom;
    dReal mass;
    dReal pos[3]; // len wid height
    box(dReal mass, dReal length,dReal width,dReal height):
        mass(mass)
    {
        pos[0]= length;
        pos[1]= width;
        pos[2]= height;
    }
};

// parameters
const dReal radius = 0.4;
const dReal mass   = 10.0;
const dReal width = 0.2;

dReal time_step= 0.01;
dReal sim_time = 0.0;

// entities
cylinder frWheel(mass,radius,width);
cylinder uplink(1.0, 0.03, 0.3);
cylinder hlink(0.5,  0.03, 0.25);
cylinder lowlink(1.0,0.03, 0.4);
box chassis(250,1.0, 0.75, 1.0); // mass length width height

const int nObs= 10;
dGeomID obstacle[nObs];
dReal obs_L=0.5;
dReal obs_W= 5.0;
dReal obs_H= 0.2;

dJointID jChassis;
dJointID jChassisUp,jChassisLow;
dJointID jRotorUp, jRotorLow, jRotorMid;
dJointID jLowSpring;


/* ========================================
 *
 * Building functions
 *
 * ======================================== */
void createWheel()
{
    dMass m1;
    frWheel.body = dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,frWheel.mass,3,frWheel.radius,frWheel.width);
    dBodySetMass(frWheel.body,&m1);
    dBodySetPosition(frWheel.body, 0.0, 0.0, frWheel.radius);
    dMatrix3 rmat;
    dRFromAxisAndAngle(rmat,1.0,0.0,0.0,  90.0*M_PI/180.0);
    dBodySetRotation(frWheel.body,rmat);

    // geom
    frWheel.geom= dCreateCylinder(carSpace,frWheel.radius,frWheel.width);
    dGeomSetBody(frWheel.geom,frWheel.body);
}
void createLinkage()
{
    dMass m1;
    dMatrix3 rmat;

    // uplink
    uplink.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetCylinderTotal(&m1,uplink.mass,3,uplink.radius,uplink.width);
    dBodySetMass(uplink.body,&m1);
    dBodySetPosition(uplink.body, 0.0, 0.5*(frWheel.width+uplink.width),
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


void createChassis()
{
    dMass m1;
    chassis.body= dBodyCreate(world);
    dMassSetZero(&m1);
    dMassSetBoxTotal(&m1,chassis.mass,chassis.pos[0],chassis.pos[1],chassis.pos[2]);
    dBodySetMass(chassis.body,&m1);
    // location
    dBodySetPosition(chassis.body,0.0,
                     0.5*(frWheel.width+chassis.pos[1])+0.8*uplink.width,2.0*frWheel.radius);
}

void buildMechanism()
{
    createWheel();
    createLinkage();
    createChassis();
}

void buildJoint()
{
    // chassis cg
    jChassis= dJointCreateSlider(world,0);
    dJointAttach(jChassis,0,chassis.body);
    dJointSetSliderAxis(jChassis, 1.0,0.0,0.0);

    dJointSetFeedback(jChassis, cgFeedback);

    // chassis pin joints
    const dReal *pos;
    jChassisUp= dJointCreateHinge(world,0);
    dJointAttach(jChassisUp,chassis.body,uplink.body);
    dJointSetHingeAxis(jChassisUp,1.0, 0.0, 0.0);
    pos= dBodyGetPosition(uplink.body);
    dJointSetHingeAnchor(jChassisUp,pos[0], pos[1]+0.5*uplink.width, pos[2]);

    const dReal *pos1;
    jChassisLow= dJointCreateHinge(world,0);
    dJointAttach(jChassisLow, chassis.body, lowlink.body);
    dJointSetHingeAxis(jChassisLow,1.0, 0.0, 0.0);
    pos1= dBodyGetPosition(lowlink.body);
    dJointSetHingeAnchor(jChassisLow, pos1[0], pos1[1]+0.5*lowlink.width, pos1[2]);

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
    jRotorUp= dJointCreateHinge(world,0);
    dJointAttach(jRotorUp, uplink.body, hlink.body);
    p2= dBodyGetPosition(uplink.body);
    dJointSetHingeAnchor(jRotorUp, p2[0], p2[1]-0.5*uplink.width, p2[2]);
    dJointSetHingeAxis(jRotorUp, 1.0, 0.0, 0.0);

    const dReal *p3;
    jRotorLow= dJointCreateHinge(world,0);
    dJointAttach(jRotorLow, lowlink.body,hlink.body);
    p3= dBodyGetPosition(lowlink.body);
    dJointSetHingeAnchor(jRotorLow, p3[0], p3[1]-0.5*lowlink.width,p3[2]);
    dJointSetHingeAxis(jRotorLow, 1.0, 0.0, 0.0);

    // robot hinge joint
    jRotorMid= dJointCreateHinge(world,0);
    dJointAttach(jRotorMid, hlink.body,frWheel.body);
    dJointSetHingeAxis(jRotorMid, 0.0,1.0,0.0);
    dJointSetHingeAnchor(jRotorMid, 0.0, 0.0, frWheel.radius);

    // suspension slider
    jLowSpring= dJointCreateLMotor(world,0);
    dJointAttach(jLowSpring, chassis.body, lowlink.body);
    dJointSetLMotorNumAxes(jLowSpring, 1);
    dJointSetLMotorAxis(jLowSpring,0,1, 0.0, 0.0, 1.0);
}

void buildObstacle()
{
    dReal xnow;
    for (int i = 0; i < nObs; ++i)
    {
        xnow= (i+1)*5.0;
        obstacle[i]= dCreateBox(groundSpace,obs_L,obs_W,obs_H);
        dGeomSetPosition(obstacle[i],xnow,0.0, 0.0);
    }
}

/* ========================================
 *
 * drawing functions
 *
 * ======================================== */
void drawWheel()
{
    const dReal *pos,*R;
    dsSetColorAlpha(1.0,0.0,0.0, 0.5);
    pos = dBodyGetPosition(frWheel.body);
    R   = dBodyGetRotation(frWheel.body);
    dsDrawCylinderD(pos,R,frWheel.width,frWheel.radius);
}

void drawLinkage()
{
    const dReal *pos1,*R1,*pos2,*R2,*pos3,*R3;
    dsSetColorAlpha(0.0,0.0,1.0, 0.5);
    pos1 = dBodyGetPosition(uplink.body);
    R1   = dBodyGetRotation(uplink.body);
    dsDrawCylinderD(pos1,R1,uplink.width,uplink.radius);

    dsSetColor(0.0,1.0,0.0);
    pos2 = dBodyGetPosition(hlink.body);
    R2   = dBodyGetRotation(hlink.body);
    dsDrawCylinderD(pos2,R2,hlink.width,hlink.radius);

    dsSetColorAlpha(1.0,0.0,1.0, 0.5);
    pos3 = dBodyGetPosition(lowlink.body);
    R3 = dBodyGetRotation(lowlink.body);
    dsDrawCylinderD(pos3,R3,lowlink.width,lowlink.radius);
}

void drawChassis()
{
    const dReal *pos, *R1;
    dsSetColorAlpha(0.3,0.8,1.0,0.5);
    pos= dBodyGetPosition(chassis.body);
    R1= dBodyGetRotation(chassis.body);
    dsDrawBoxD(pos,R1,chassis.pos);
}

void drawObstacle()
{
    const dReal *pos, *R1;
    double sides[3];
    sides[0]= obs_L;
    sides[1] = obs_W;
    sides[2] = obs_H;

    dsSetColor(0.5,0.5,0.2);
    for (int i = 0; i < nObs; ++i)
    {
        pos= dGeomGetPosition(obstacle[i]);
        R1= dGeomGetRotation(obstacle[i]);
        dsDrawBoxD(pos,R1,sides);
    }
}

/* =========================================
 *
 * Sim functions
 *
 *=========================================== */
void simJoint()
{
    dReal kd= 2;
    dReal kp= 5;
    dReal erp= time_step*kp/(time_step*kp+kd);
    dReal cfm= 1/(time_step*kp+kd);

    dJointSetLMotorParam(jLowSpring, dParamHiStop, 0.0);
    dJointSetLMotorParam(jLowSpring, dParamLoStop, 0.0);
    dJointSetLMotorParam(jLowSpring, dParamStopCFM, cfm);
    dJointSetLMotorParam(jLowSpring, dParamStopERP, erp);
    dJointSetLMotorParam(jLowSpring, dParamFMax, 10);

}

void simControl()
{
    //dJointSetHingeParam(jRotorMid, dParamVel, -1.0*M_PI);
    //dJointSetHingeParam(jRotorMid, dParamFMax, 100);

    dJointSetSliderParam(jChassis, dParamVel, -0.5);
    dJointSetSliderParam(jChassis, dParamFMax, 1000);
}

void simFeedback(dReal sim_time)
{
    \
    cgFeedback = dJointGetFeedback(jChassis);
    printf("%8.4f Force fx=%6.2f ", sim_time,cgFeedback->f1[0]);
    printf("fy=%6.2f ",cgFeedback->f1[1]);
    printf("fz=%6.2f \n",cgFeedback->f1[2]);

}

/* =========================================
 *
 * Major functions
 *
 *=========================================== */
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

static void simLoop (int pause)
{

    dSpaceCollide(space,0,&nearCallback);
    simJoint();
    simControl();


    dWorldStep(world,time_step);
    dJointGroupEmpty(contactGroup);

    sim_time += time_step;
    simFeedback(sim_time);

    drawLinkage();
    drawWheel();
    drawChassis();
    drawObstacle();

    // terminiate
    const dReal *pos= dBodyGetPosition(chassis.body);
    if (pos[0]>10.0)
        dsStop();
}

void start()
{
    static float xyz[3] = {-3.0,1.0,3.0};
    static float hpr[3] = {-15.0,-15.0,0.0};
    dsSetViewpoint (xyz,hpr);
}




int main (int argc, char **argv)
{
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = NULL;
    fn.stop    = NULL;
    fn.path_to_textures = "C:\\dev\\ode-0.13\\drawstuff\\textures";

    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world,0,0, -9.81);

    space= dHashSpaceCreate(0);
    carSpace= dHashSpaceCreate(space);
    groundSpace= dHashSpaceCreate(space);
    dSpaceSetSublevel(groundSpace,1);
    dSpaceSetSublevel(carSpace,1);


    // create ground
    ground= dCreatePlane(groundSpace,0.0, 0.0, 1.0, 0.0);  // z= 0 plane

    buildObstacle();

    buildMechanism();
    buildJoint();


    dsSimulationLoop (argc,argv,900,750,&fn);

    dWorldDestroy (world);
    dCloseODE();

    return 0;
}
