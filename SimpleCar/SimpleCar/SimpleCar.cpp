// SimpleCar.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
// Simple ODE car simulator
#include <fstream>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <vmCar.h>
#include <vmWishboneCar.h>


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
dReal STEPSIZE = 0.01;
const int maxContacts = 10;

// ground and obstacle balls
dGeomID ground;
const int nObs = 10;
sphere ball[nObs];
box obs[nObs];

// chassis parameters
dReal chassisMass = 1300;
dReal chassisLength = 2.0;
dReal chassisWidth = 1.5;
dReal chassisHeight = 0.75;

// wheel paremeters
dReal wheelMass = 10;
dReal wheelRadius = 0.4;  // wheel radius
dReal wheelWidth = 0.2;  // wheel width
dReal kps = 55000; //55000.0;  // suspension stiffness
dReal kds = 1579;   //4000.0; // 4000.0;  //1579.0;   // suspension damping

					// control parameters
dReal speed = 0.0;
dReal steer = 0.0;
dReal steerGain = 100;

// control mode flags
int manualYes = 0;
int brakeYes = 0;

// output file
std::ofstream fp_pos;
std::ofstream fp_jnt;

// collision callback
static void nearCallback(void *data, dGeomID g1, dGeomID g2)
{
	if (dGeomIsSpace(g1) && dGeomIsSpace(g2))
	{
		// colliding a space with something
		dSpaceCollide2(g1, g2, data, &nearCallback);
	}
	else
	{
		// check contacts
		dContact contact[maxContacts];
		int nContact = dCollide(g1, g2, maxContacts, &contact[0].geom, sizeof(dContact));

		// treat contacts
		if (nContact>0)
		{
			for (int i = 0; i<nContact; ++i)
			{
				contact[i].surface.mode = dContactApprox1_1 | dContactApprox1_2
					| dContactSoftCFM | dContactSoftERP;

				contact[i].surface.mu = 1;
				contact[i].surface.mu2 = 1;
				//contact[i].surface.slip1 = 1e-10;
				//contact[i].surface.slip2 = 1e-10;
				contact[i].surface.soft_erp = 0.01;
				contact[i].surface.soft_cfm = 1e-5;

				// build contact joint now
				dJointID c = dJointCreateContact(world, contactGroup, &contact[i]);
				dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
				//dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));

			}
		}
	}
}


// simulation loop
static void simloop(int pause)
{

	static dReal simCt;

	// update suspension

	// nonlinear block
	/*dReal nl_kd;
	nl_kd= car->getNonlinearKd(vm::FR,STEPSIZE);
	car->setWheelSuspension(vm::FR,STEPSIZE,kps,nl_kd);
	nl_kd= car->getNonlinearKd(vm::FL,STEPSIZE);
	car->setWheelSuspension(vm::FL,STEPSIZE,kps,nl_kd);
	nl_kd= car->getNonlinearKd(vm::RR,STEPSIZE);
	car->setWheelSuspension(vm::RR,STEPSIZE,kps,nl_kd);
	nl_kd= car->getNonlinearKd(vm::RL,STEPSIZE);
	car->setWheelSuspension(vm::RL,STEPSIZE,kps,nl_kd);*/


	car->setAllWheelSuspension(STEPSIZE, kps, kds);

	//set control
	//car->simControl();
	car->simForward(20.0);

	dSpaceCollide(space, 0, &nearCallback);
	dWorldStep(world, STEPSIZE);
	simCt++;

	// collision setup
	dJointGroupEmpty(contactGroup);

	// draw car now
	car->simDraw();

	// draw obstacles
	dsSetColor(0.0, 1.0, 0.0);
	for (int i = 0; i<nObs; ++i)
	{
		//dsDrawSphere(dGeomGetPosition(ball[i].geom),dGeomGetRotation(ball[i].geom),ball[i].radius);
		dsDrawBoxD(dGeomGetPosition(obs[i].geom)
			, dGeomGetRotation(obs[i].geom), obs[i].sides);
	}

	// report
	car->listVehiclePosition(fp_pos, simCt, STEPSIZE);
	car->listWheelForce(fp_jnt, simCt, STEPSIZE);

	//printf("\n");


	// termination condition
	const dReal *pos = dBodyGetPosition(car->chassis.body);
	if (pos[0]>60.0)
		dsStop();
}

// command
void command(int cmd)
{
	car->simCommand(cmd);
}

// camera setup
void start()
{
	static float xyz[3] = { -10.0,-5.0,5.0 };
	static float hpr[3] = { 5.0,-15.0,0.0 };
	dsSetViewpoint(xyz, hpr);
}

// drawstuff initialization
void setDSFunctions()
{
	fn.version = DS_VERSION;
	fn.step = &simloop;
	fn.start = &start;
	fn.stop = NULL;
	fn.command = &command;
	fn.path_to_textures = texturePath;
	//fn.path_to_textures= "c:/dev/ode-0.13/drawstuff/textures";  // win version
	//fn.path_to_textures= "/home/jlo/ode-0.13/drawstuff/textures";  // linux version
}

// create a standard car
void createCar()
{
	car = new vmCar(world, vehicleSpace);
	car->setChassis(chassisMass, chassisLength,
		chassisWidth, chassisHeight);
	car->setAllWheel(wheelMass, wheelWidth, wheelRadius);
	car->setCarOnGround(0.0, 0.0);
	car->setAllWheelJoint();
	car->setInitialControls(steer, speed, steerGain);
}

// create a wishbone suspension car
void createWishboneCar()
{
	car = new vmWishboneCar(world, vehicleSpace);
	car->setChassis(chassisMass, chassisLength,
		chassisWidth, chassisHeight);
	car->setAllWheel(wheelMass, wheelWidth, wheelRadius);
	car->setCarOnGround(0.0, 0.0);
	car->setAllWheelJoint();
	car->setInitialControls(steer, speed, steerGain);
}


// create obstacles
void createObstacles()
{
	dReal xo, yo;

	for (int i = 0; i<nObs; ++i)
	{
		xo = dRandReal() * 50;
		yo = dRandReal() * 50;
		ball[i].radius = 1.0;
		ball[i].geom = dCreateSphere(groundSpace, ball[i].radius);
		dGeomSetPosition(ball[i].geom, xo, -yo, -0.9);
	}
}

// create aligned ball obstacles
void createAlignedObstacles()
{
	dReal xo, yo;

	for (int i = 0; i<nObs; ++i)
	{
		xo = (i + 1)*5.0;
		yo = chassisWidth*0.5;
		ball[i].radius = 2;
		ball[i].geom = dCreateSphere(groundSpace, ball[i].radius);
		dGeomSetPosition(ball[i].geom, xo, yo, -1.8);
		//dMatrix3 rmat;
		//dRFromAxisAndAngle(rmat,1.0, 0.0, 0.0, 90.0*M_PI/180.0);

		//dGeomSetRotation(bumps[i].geom, rmat);
	}
}

// create Box obstacles
void createBoxObstacles()
{
	dReal xo, yo;

	for (int i = 0; i<nObs; ++i)
	{
		xo = (i + 1)*5.0;
		yo = chassisWidth*0.5;
		obs[i].sides[0] = 0.5; //length
		obs[i].sides[1] = 5;   // width
		obs[i].sides[2] = 0.2;  //height
		obs[i].geom = dCreateBox(groundSpace, obs[i].sides[0],
			obs[i].sides[1], obs[i].sides[2]);

		dGeomSetPosition(obs[i].geom, xo, yo, 0.0);
		//dMatrix3 rmat;
		//dRFromAxisAndAngle(rmat,1.0, 0.0, 0.0, 90.0*M_PI/180.0);

		//dGeomSetRotation(bumps[i].geom, rmat);
	}
}

// main
int main(int argc, char **argv)
{
	setDSFunctions();
	dInitODE();
	// create basis
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	vehicleSpace = dHashSpaceCreate(space);
	groundSpace = dHashSpaceCreate(space);
	dSpaceSetSublevel(groundSpace, 1);
	dSpaceSetSublevel(vehicleSpace, 1);

	contactGroup = dJointGroupCreate(0);

	//set gravity
	dWorldSetGravity(world, 0.0, 0.0, -9.81);

	// create ground
	ground = dCreatePlane(groundSpace, 0.0, 0.0, 1.0, 0.0);  // z= 0 plane
															 // create obstacles
															 //createAlignedObstacles();
	createBoxObstacles();

	// create car
	//createCar();
	createWishboneCar();


	// output files
	fp_pos.open("vehiclePos.txt");
	fp_jnt.open("wheelForce.txt");

	// start simulation
	dsSimulationLoop(argc, argv, 1000, 800, &fn);

	fp_pos.close();
	fp_jnt.close();

	dWorldDestroy(world);
	dSpaceDestroy(space);
	dCloseODE();

	delete car;

	return 0;
}
