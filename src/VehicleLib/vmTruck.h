#ifndef VMTRUCK_H
#define VMTRUCK_H

#include <fstream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "vmCar.h"


class vmTractor
{
public:
	vmTractor(dWorldID world, dSpaceID space);
	~vmTractor();

	dWorldID world;
	dSpaceID space;

	vmChassis chassis;

	// tractor wheels
	vmWheel tractor_r1Wheel; // front axle
	vmWheel tractor_l1Wheel;
	vmWheel tractor_r2Wheel; // second axle
	vmWheel tractor_l2Wheel;
	vmWheel tractor_r3Wheel; // thrid axle
	vmWheel tractor_l3Wheel;

	// trailer wheels
	vmWheel trailer_r1Wheel;
	vmWheel trailer_l1Wheel;
	vmWheel trailer_r2Wheel;
	vmWheel trailer_l2Wheel;

	// control flags
	int manualYes{ 0 }; // manual control flag
	int brakeYes{ 0 }; // brake control flag

	// parameters
	dReal speed{ 0.0 };
	dReal steer{ 0.0 };
	dReal steerGain{ 0.0 };

	// setter
	void setTractorChassis(dReal mass, dReal length, dReal width, dReal height);
	void setTrailerChassis(dReal mass, dReal length, dReal width, dReal height);

	void setWheel(vm::WheelLoc loc, dReal mass, dReal length, dReal radius);
	void setAllWheel(dReal mass, dReal length, dReal radius);

	void setCMPosition(dReal x, dReal y, dReal z);
	void setCarOnGround(dReal x, dReal y);

	virtual void setWheelJoint(vm::WheelLoc loc);
	void setAllWheelJoint();

	virtual void setWheelSuspension(vm::WheelLoc loc, dReal step, dReal kps, dReal kds);
	void setAllWheelSuspension(dReal step, dReal kps, dReal kds);

	// simloop functions
	void simCommand(int cmd);
	virtual void simControl();
	virtual void simForward(dReal mphSpeed, dReal targetSteer = 0);
	//void simSlowSteer();

	// simloop draw function
	virtual void simDraw();

	// control funcitons
	void setInitialControls(dReal steer, dReal speed, dReal steerGain);

	// suspension functions
	dReal getNonlinearKd(vm::WheelLoc loc, dReal step);
	virtual dReal getSuspensionRate(vm::WheelLoc loc, dReal step);

	// getter
	dReal getTotalMass();

	// report functions
	void listVehiclePosition(std::ofstream &fp, dReal simCt, dReal step);
	void listWheelForce(std::ofstream &fp, dReal simCt, dReal step);


protected:
	// compute ERP and CFM
	dReal computeERP(dReal step, dReal kp, dReal kd);
	dReal computeCFM(dReal step, dReal kp, dReal kd);
	dReal bounded(dReal var, dReal lb, dReal ub);

private:

};

class vmTractor
{
    public:
        vmTractor(dWorldID world,dSpaceID space);
        ~vmTractor();

        dWorldID world;
        dSpaceID space;

        vmChassis chassis;
		
		// tractor wheels
        vmWheel tractor_r1Wheel; // front axle
        vmWheel tractor_l1Wheel;
        vmWheel tractor_r2Wheel; // second axle
		vmWheel tractor_l2Wheel;
		vmWheel tractor_r3Wheel; // thrid axle
		vmWheel tractor_l3Wheel;

		// trailer wheels
		vmWheel trailer_r1Wheel;
		vmWheel trailer_l1Wheel;
		vmWheel trailer_r2Wheel;
		vmWheel trailer_l2Wheel;

		// control flags
        int manualYes {0}; // manual control flag
        int brakeYes {0}; // brake control flag

		// parameters
        dReal speed {0.0};
        dReal steer {0.0};
        dReal steerGain {0.0};
		
        // setter
        void setTractorChassis(dReal mass, dReal length, dReal width, dReal height);
		void setTrailerChassis(dReal mass, dReal length, dReal width, dReal height);

        void setWheel(vm::WheelLoc loc,dReal mass,dReal length,dReal radius);
        void setAllWheel(dReal mass,dReal length,dReal radius);

        void setCMPosition(dReal x, dReal y, dReal z);
        void setCarOnGround(dReal x, dReal y);

        virtual void setWheelJoint(vm::WheelLoc loc);
        void setAllWheelJoint();

        virtual void setWheelSuspension(vm::WheelLoc loc,dReal step,dReal kps, dReal kds);
        void setAllWheelSuspension(dReal step,dReal kps, dReal kds);

        // simloop functions
        void simCommand(int cmd);
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


    protected:
        // compute ERP and CFM
        dReal computeERP(dReal step, dReal kp, dReal kd);
        dReal computeCFM(dReal step, dReal kp, dReal kd);
        dReal bounded(dReal var, dReal lb, dReal ub);

    private:

};



#endif // VMTRUCK_H
