#ifndef VMCAR_H
#define VMCAR_H

#include <vector>
#include <fstream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

struct GeomPair
{
	dGeomID geomT; // geometry transform
	dGeomID geomEnc; // encapsulated geometry
};

// vehicle component definition
struct vmChassis{
    dBodyID body;
	std::vector<GeomPair> geoms;
    dReal mass;
    dReal sides[3];  //length,width,height
    bool initialized= false;

    vmChassis(): body(0),mass(0.0)
    {
		geoms.resize(1);		
	}
	
	void SetProperty(const dWorldID &world, const dSpaceID &space, 
		dReal mass, dReal length, dReal width, dReal height);

};

struct vmWheel{
    dBodyID body;
    GeomPair geom;
    dJointID joint;
    dJointFeedback *feedback;
    dJointID motor;  //aMotor
    dJointID brake;  //aMotor
    dReal mass;
    dReal length,radius;
    bool initialized= false;
    dReal dzSuspension=0.0;

	vmWheel() : body(0), joint(0), motor(0), mass(0),
		length(0), radius(0) {
		feedback = new dJointFeedback;
		geom.geomT = 0;
		geom.geomEnc = 0;
	}
	~vmWheel()
	{
		delete feedback;
	}
	void SetProperty(const dWorldID &world, const dSpaceID &space,
		dReal mass, dReal length, dReal radius);
};

// constants
namespace vm {
    enum SIDE {RIGHT,LEFT};
}

class vmCar
{
    public:
        vmCar(dWorldID world,dSpaceID space, int num_axles);
        virtual ~vmCar();

        dWorldID world_;
        dSpaceID space_;

        vmChassis chassis_;
		std::vector<vmWheel> left_wheels_;
		std::vector<vmWheel> right_wheels_;

        bool manual_act_ {false}; // manual control flag
        bool brake_act_ {false}; // brake control flag

        dReal speed {0.0};
        dReal steer {0.0};
        dReal steerGain {0.0};


        // setter
        void SetChassis(dReal mass, dReal length, dReal width, dReal height);

        void SetWheel(vm::SIDE side, int idx, dReal mass,dReal length,dReal radius);
        void SetAllWheel(dReal mass,dReal length,dReal radius);

		// Set vehicle CM position
		// @side: left or right side
		// @axleIdx: axle index
		// @x_vf: wheel x position in vehicle frame
		// @y_vf: wheel y position in vehicle frame
		// @z_vf: wheel z position in vehicle frame
        void SetWheelPosition(vm::SIDE side, int axleIdx, 
			dReal x_vf, dReal y_vf, dReal z_vf);
		void SetCMPosition(dReal x, dReal y, dReal z);
        void SetCarOnGround(dReal x, dReal y);

        virtual void SetWheelJoint(vm::SIDE side, int idx, bool steerable=false);
        /* set joints for all wheels
		@steerable_axles: steerable axle indices vector. if empty, the steerable 
		axle will be 0 by default
		*/
		void SetAllWheelJoint(const std::vector<int> steerable_axles);

		/* set wheel suspension properties
			@side: left or right side
			@idx: index of the axle
			@step: simulatio time step
			@kps: stiffness
			@kds: damping
		*/
        virtual void SetWheelSuspension(vm::SIDE side, int idx,dReal step,dReal kps, dReal kds);
        void SetAllWheelSuspension(dReal step,dReal kps, dReal kds);

        // simloop functions
        void SimCommand(int cmd);
        virtual void simControl();
        virtual void simForward(dReal mphSpeed, dReal targetSteer= 0);
        //void simSlowSteer();

        // simloop draw function
        virtual void simDraw();

        // control funcitons
        void setInitialControls(dReal steer,dReal speed,dReal steerGain);

        // suspension functions
        dReal getNonlinearKd(vm::WheelLoc loc, dReal step);
        virtual dReal getSuspensionRate(vm::WheelLoc loc, dReal step);

        // getter
        dReal getTotalMass();

        // report functions
        void listVehiclePosition(std::ofstream &fp, dReal simCt, dReal step);
        void listWheelForce(std::ofstream &fp, dReal simCt, dReal step);

    private:

};

// utility functions
namespace vm
{
	// compute ERP and CFM
	dReal ComputeERP(dReal step, dReal kp, dReal kd);
	dReal ComputeCFM(dReal step, dReal kp, dReal kd);
	dReal Bounded(dReal var, dReal lb, dReal ub);
}




#endif // VMCAR_H
