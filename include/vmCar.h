#ifndef VMCAR_H
#define VMCAR_H

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

struct vmChassis{
    dBodyID body;
    dGeomID geom;
    dReal mass;
    dReal sides[3];  //length,width,height
    bool initialized= false;

    //vmChassis(): body(0),geom(0),mass(0.0)
    //{}
};

struct vmWheel{
    dBodyID body;
    dGeomID geom;
    dJointID joint;
    dJointFeedback *feedback;
    dJointID motor;  //aMotor
    dJointID brake;  //aMotor
    dReal mass;
    dReal length,radius;
    bool initialized= false;
    dReal dzSuspension=0.0;

    //vmWheel(): body(0),geom(0),joint(0),motor(0),mass(0),
    //    length(0),radius(0)
    //{}
};

namespace vm {
    enum WheelLoc {FR,FL,RR,RL};
}

class vmCar
{
    public:
        vmCar(dWorldID world,dSpaceID space);
        virtual ~vmCar();

        dWorldID world;
        dSpaceID space;

        vmChassis chassis;
        vmWheel frWheel;
        vmWheel flWheel;
        vmWheel rrWheel;
        vmWheel rlWheel;

        int manualYes; // manual control flag
        int brakeYes; // brake control flag

        dReal speed;
        dReal steer;
        dReal steerGain;


        // setter
        void setChassis(dReal mass, dReal length, dReal width, dReal height);

        void setWheel(vm::WheelLoc loc,dReal mass,dReal length,dReal radius);
        void setAllWheel(dReal mass,dReal length,dReal radius);

        void setCMPosition(dReal x, dReal y, dReal z);
        void setCarOnGround(dReal x, dReal y);

        void setWheelJoint(vm::WheelLoc loc);
        void setAllWheelJoint();

        void setWheelSuspension(vm::WheelLoc loc,dReal step,dReal kps, dReal kds);
        void setAllWheelSuspension(dReal step,dReal kps, dReal kds);

        // simloop functions
        void simCommand(int cmd);
        void simControl();
        void simForward(dReal mphSpeed);
        void simSlowSteer();

        // simloop draw function
        void simDraw();

        // control funcitons
        void setInitialControls(dReal steer,dReal speed,dReal steerGain);

        // getter
        dReal getTotalMass();
        dReal getNonlinearKd(vm::WheelLoc loc, dReal step);


        // report functions
        void listVehiclePosition(FILE *fp, dReal simCt, dReal step);
        void listWheelForce(FILE *fp, dReal simCt, dReal step);

protected:
    private:
        // compute ERP and CFM
        dReal computeERP(dReal step, dReal kp, dReal kd);
        dReal computeCFM(dReal step, dReal kp, dReal kd);
        dReal bounded(dReal var, dReal lb, dReal ub);
};

struct sphere{
    dGeomID geom;
    dReal radius;
};

struct cylinder{
    dGeomID geom;
    dReal radius;
    dReal length;
};

struct box{
    dGeomID geom;
    dReal sides[3]; // length width height
};



#endif // VMCAR_H
