#ifndef VMCAR_H
#define VMCAR_H

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

struct vmChassis{
    dBodyID body;
    dGeomID geom;
    dReal mass;
    dReal sides[3];  //length,width,height

    vmChassis(): body(0),geom(0),mass(0.0)
    {}
};

struct vmWheel{
    dBodyID body;
    dGeomID geom;
    dJointID joint;
    dJointID motor;
    dReal mass;
    dReal length,radius;

    vmWheel(): body(0),geom(0),joint(0),motor(0),mass(0),
        length(0),radius(0)
    {}
};

namespace vmCar {
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

        // setter
        void setChassis(dReal mass, dReal length, dReal width, dReal height);

        void setWheel(vmCar::WheelLoc loc,dReal mass,dReal length,dReal radius);
        void setAllWheel(dReal mass,dReal length,dReal radius);

        void setCMPosition(dReal x, dReal y, dReal z);
        void setCarOnGround(dReal x, dReal y);

        void setWheelJoint(vmCar::WheelLoc loc,dReal kps, dReal kds);
        void setAllWheelJoint(dReal kps, dReal kds);



    protected:
    private:
        // compute ERP and CFM
        dReal computeERP(dReal kp, dReal kd);
        dReal computeCFM(dReal kp, dReal kd);
        dReal bounded(dReal var, dReal lb, dReal ub);

};

struct sphere{
    dGeomID geom;
    dReal radius;
};



#endif // VMCAR_H
