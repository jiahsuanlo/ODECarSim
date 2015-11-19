#ifndef VMCAR_H
#define VMCAR_H


typedef struct{
    dBodyID body;
    dGeomID geom;
    dReal mass;
    dReal sides[3];  //length,width,height
} vmChassis;

typedef struct {
    dBodyID body;
    dGeomID geom;
    dJointID joint;
    dJointID motor;
    dReal mass;
    dReal length,radius;
} vmWheel;

class vmCar
{
    public:
        vmCar();
        virtual ~vmCar();

        vmChassis chassis;
        vmWheel frWheel;
        vmWheel flWheel;
        vmWheel rrWheel;
        vmWheel rlWheel;

        // setter
        void setChassisDim(dReal length, dReal width, dReal Height);

    protected:
    private:
};

#endif // VMCAR_H
