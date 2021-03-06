#include "vmCar.h"
#include <vector>

vmCar::vmCar(dWorldID world, dSpaceID space,int num_axles)
{
    this->world_= world;
    this->space_= space;
	left_wheels_.resize(num_axles);
	right_wheels_.resize(num_axles);	
}


vmCar::~vmCar()
{
    //delete left side wheel feedback
	for (auto &wh : left_wheels_)
	{
		delete wh.feedback;
	}
	// delete right side wheel feedback
	for (auto &wh : right_wheels_)
	{
		delete wh.feedback;
	}
}

/* set chassis dimension length, width, and height */
void vmChassis::SetProperty(const dWorldID &world, const dSpaceID &space,
	dReal mass, dReal length, dReal width, dReal height)
{
    sides[0]= length;
    sides[1]= width;
    sides[2]= height;
    this->mass= mass;

    // create body
	body= dBodyCreate(world);
    
	// set up geometry (assume only one geom for now)
	geoms[0].geomEnc= dCreateBox(0,sides[0],sides[1],sides[2]);
	geoms[0].geomT = dCreateGeomTransform(space);
	dGeomTransformSetGeom(geoms[0].geomT, geoms[0].geomEnc);
	dGeomTransformSetInfo(geoms[0].geomT, 1); 

    // mass properties
    dMass m1;
    dMassSetZero(&m1);
    dMassSetBoxTotal(&m1,mass,sides[0],sides[1],sides[2]);
    dBodySetMass(body,&m1);
    dGeomSetBody(geoms[0].geomT,body);

    // set default controls
    this->steer= 0.0;
    this->speed= 0.0;
    this->steerGain= 100;

    initialized=true;
}


void vmCar::SetWheelPosition(vm::SIDE side, int axleIdx,
	dReal x_vf, dReal y_vf, dReal z_vf)
{
	vmWheel *wnow;
	if (side == vm::LEFT)
		wnow = &left_wheels_[axleIdx];
	else
		wnow = &right_wheels_[axleIdx];
	
	// if any of chassis and wheels are not setup, return
    if ((!chassis_.initialized) || (!wnow->initialized) )
    {
        printf("setCM: Some chassis/wheel entity is not defined yet!");
        return;
    }

	// get wheel global position
	dVector3 wpos;
	dBodyGetRelPointPos(chassis_.body, x_vf, y_vf, z_vf, wpos);
	
    // set wheel position
	dBodySetPosition(wnow->body,wpos[0], wpos[1], wpos[2]);

    //set wheel orientation
    const dReal *rmat= dBodyGetRotation(chassis_.body);
    dBodySetRotation(wnow->body, rmat);
}

void vmCar::SetCMPosition(dReal x, dReal y, dReal z)
{
	// check to ensure all left wheels are initialized
	bool left_whl_initialized = true;
	for (const auto &wh : left_wheels_)
	{
		if (!wh.initialized)
		{
			left_whl_initialized = false;
			break;
		}
	}
	// check to ensure all right wheels are initialized
	bool right_whl_initialized = true;
	for (const auto &wh : right_wheels_)
	{
		if (!wh.initialized)
		{
			right_whl_initialized = false;
			break;
		}
	}

	// if any of chassis and wheels are not setup, return
	if ((!chassis_.initialized) || (!left_whl_initialized) ||
		(!right_whl_initialized))
	{
		printf("setCM: Some chassis/wheel entity is not defined yet!");
		return;
	}

	// get wheel positions in vehicle frame
	std::vector<dVector3> lt_whl_pos_vf;
	std::vector<dVector3> rt_whl_pos_vf;
	for (auto &wh : left_wheels_)
	{
		dVector3 pos_vf;
		const dReal *wh_pos = dBodyGetPosition(wh.body);
		dBodyGetPosRelPoint(chassis_.body, wh_pos[0], wh_pos[1], wh_pos[2], pos_vf);
		lt_whl_pos_vf.push_back(pos_vf);
	}
	for (auto &wh : right_wheels_)
	{
		dVector3 pos_vf;
		const dReal *wh_pos = dBodyGetPosition(wh.body);
		dBodyGetPosRelPoint(chassis_.body, wh_pos[0], wh_pos[1], wh_pos[2], pos_vf);
		rt_whl_pos_vf.push_back(pos_vf);
	}

	// set chassis position
	dBodySetPosition(chassis_.body, x, y, z);

	// set wheel positions
	for (size_t i=0; i<left_wheels_.size(); i++)
	{
		// obtain global position of the wheel
		dVector3 wh_pos;
		dBodyGetRelPointPos(chassis_.body,
			lt_whl_pos_vf[i][0], lt_whl_pos_vf[i][1], lt_whl_pos_vf[i][2],
			wh_pos);
		// set wheel position now
		dBodySetPosition(left_wheels_[i].body, wh_pos[0], wh_pos[1], wh_pos[2]);		
	}
	for (size_t i = 0; i < right_wheels_.size(); i++)
	{
		// obtain global position of the wheel
		dVector3 wh_pos;
		dBodyGetRelPointPos(chassis_.body,
			rt_whl_pos_vf[i][0], rt_whl_pos_vf[i][1], rt_whl_pos_vf[i][2],
			wh_pos);
		// set wheel position now
		dBodySetPosition(right_wheels_[i].body, wh_pos[0], wh_pos[1], wh_pos[2]);
	}
}
/* place the car on the ground based on the x and y location
    z location will be automatically determined
	assuming the ground level is at 0
*/
void vmCar::SetCarOnGround(dReal x, dReal y)
{
    dReal znow;
    znow= left_wheels_[0].radius + 0.5*chassis_.sides[2];
	SetCMPosition(x, y, znow);
}

void vmCar::SetAllWheelJoint(const std::vector<int> steerable_axles)
{
	size_t num_axles = left_wheels_.size();
	size_t num_steerable_axles = steerable_axles.size();
	if (num_steerable_axles > num_axles)
	{
		const char* errstr = "set joint: the number of steerable axles must not exceed the number of axles";
		printf(errstr);
		throw std::invalid_argument(errstr);
		return;
	}

	// check to ensure all left wheels are initialized
	bool left_whl_initialized = true;
	for (const auto &wh : left_wheels_)
	{
		if (!wh.initialized)
		{
			left_whl_initialized = false;
			break;
		}
	}
	// check to ensure all right wheels are initialized
	bool right_whl_initialized = true;
	for (const auto &wh : right_wheels_)
	{
		if (!wh.initialized)
		{
			right_whl_initialized = false;
			break;
		}
	}
    // if any of chassis and wheels are not setup, return
    if ((!chassis_.initialized) || (!left_whl_initialized) ||
            (!right_whl_initialized))
    {
		char* errstr = "set Joint: Some chassis/wheel entity is not defined yet!";
        printf(errstr);
		throw std::runtime_error(errstr);
    }

	// convert steerable_axles to bool vector
	std::vector<bool> steerable(num_axles,false);
	for (size_t i = 0; i < num_steerable_axles; i++)
	{
		int idx = steerable_axles[i];
		if (idx >= num_axles)
		{
			char *errstr = "set joint: the steerable axle number exceeds the number of all axles";
			printf(errstr);
			throw std::invalid_argument(errstr);
		}
		steerable[idx] = true;
	}

	// set joint for the wheels
	for (size_t i = 0; i < num_axles; i++)
	{
		if (steerable[i]) // steerable axle
		{
			SetWheelJoint(vm::LEFT, i, true);
			SetWheelJoint(vm::RIGHT, i, true);
		}
		else // non steerable axle
		{
			SetWheelJoint(vm::LEFT, i, false);
			SetWheelJoint(vm::RIGHT, i, false);
		}		
	}

}

/* simloop control subfunction
    inputs are control mode flags
    int manualYes: 1: under manual input; 0 otherwise
    int brakeYes: 1: brake activated; 0 otherwise
*/
void vmCar::simControl()
{
    // auto align steering angle
    if (manualYes)
        steerGain= 10;
    else
    {
        steerGain= 10;
        steer*= 0.98;
        speed*= 0.999;
    }


    dReal dsteer, realSpeed;
    dsteer = bounded(steer,-40.0,40.0) - dJointGetHinge2Angle1(frWheel.joint);
    realSpeed = -bounded(speed, -14*M_PI, 30*M_PI);
    // steer
    dJointSetHinge2Param(frWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(frWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (frWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamFudgeFactor,0.1);

    dJointSetHinge2Param(flWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(flWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (flWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamFudgeFactor,0.1);
    // speed
    if (brakeYes)
    {
        dReal factor= 0.1;
        dJointSetHinge2Param(rrWheel.joint, dParamVel2, dJointGetHinge2Param(rrWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(rrWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(rlWheel.joint, dParamVel2, dJointGetHinge2Param(rlWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(rlWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(frWheel.joint, dParamVel2, realSpeed);//dJointGetHinge2Param(frWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(frWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(flWheel.joint, dParamVel2, realSpeed);//dJointGetHinge2Param(flWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(flWheel.joint, dParamFMax2, 1000.0);
    }
    else
    {
        // turn off rear wheels
        dJointSetHinge2Param(rrWheel.joint, dParamFMax2, 0.0);
        dJointSetHinge2Param(rlWheel.joint, dParamFMax2, 0.0);
        // set up front wheels
        dJointSetHinge2Param(frWheel.joint, dParamVel2, realSpeed);
        dJointSetHinge2Param(frWheel.joint, dParamFMax2, 500.0);
        dJointSetHinge2Param(flWheel.joint, dParamVel2, realSpeed);
        dJointSetHinge2Param(flWheel.joint, dParamFMax2, 500.0);
    }

    // reset manual mode flag
    manualYes= 0;

}

void vmCar::simForward(dReal mphSpeed, dReal targetSteer)
{
    dReal dsteer, realSpeed;
    dsteer = targetSteer - dJointGetHinge2Angle1(frWheel.joint);  // target at zero steer

    // calculate wheel real speed
    dReal siSpeed= mphSpeed*1.6/3600.0*1000; // convert ot m/s
    realSpeed = -siSpeed/frWheel.radius;
	
    // steer
    dJointSetHinge2Param(frWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(frWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (frWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (frWheel.joint,dParamFudgeFactor,0.1);

    dJointSetHinge2Param(flWheel.joint, dParamVel, steerGain*dsteer+0.01*dsteer/0.01);
    dJointSetHinge2Param(flWheel.joint, dParamFMax, 1000.0);
    dJointSetHinge2Param (flWheel.joint,dParamLoStop,-40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamHiStop,40.0*M_PI/180.0);
    dJointSetHinge2Param (flWheel.joint,dParamFudgeFactor,0.1);
    // speed
    if (brakeYes)
    {
        dReal factor= 0.1;
        dJointSetHinge2Param(rrWheel.joint, dParamVel2, dJointGetHinge2Param(rrWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(rrWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(rlWheel.joint, dParamVel2, dJointGetHinge2Param(rlWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(rlWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(frWheel.joint, dParamVel2, realSpeed);//dJointGetHinge2Param(frWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(frWheel.joint, dParamFMax2, 1000.0);
        dJointSetHinge2Param(flWheel.joint, dParamVel2, realSpeed);//dJointGetHinge2Param(flWheel.joint,dParamVel2)*factor);
        dJointSetHinge2Param(flWheel.joint, dParamFMax2, 1000.0);
    }
    else
    {
        // turn off rear wheels
        dJointSetHinge2Param(rrWheel.joint, dParamFMax2, 0.0);
        dJointSetHinge2Param(rlWheel.joint, dParamFMax2, 0.0);
        // set up front wheels
        dJointSetHinge2Param(frWheel.joint, dParamVel2, realSpeed);
        dJointSetHinge2Param(frWheel.joint, dParamFMax2, 500.0);
        dJointSetHinge2Param(flWheel.joint, dParamVel2, realSpeed);
        dJointSetHinge2Param(flWheel.joint, dParamFMax2, 500.0);
    }

}

void vmCar::simDraw()
{
    // draw chassis
    dsSetColor(1.0,0.0,0.0); // red
    dsDrawBoxD(dBodyGetPosition(chassis.body),dBodyGetRotation(chassis.body)
               ,chassis.sides);
    // draw wheels
    dsSetColor(1.0,1.0,0.0);
    dsDrawCylinderD(dBodyGetPosition(frWheel.body),dBodyGetRotation(frWheel.body),frWheel.length,frWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(flWheel.body),dBodyGetRotation(frWheel.body),flWheel.length,flWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(rrWheel.body),dBodyGetRotation(rrWheel.body),rrWheel.length,rrWheel.radius);
    dsDrawCylinderD(dBodyGetPosition(rlWheel.body),dBodyGetRotation(rlWheel.body),rlWheel.length,rlWheel.radius);

}

void vmCar::setInitialControls(dReal steer, dReal speed, dReal steerGain)
{
    this->steer= steer;
    this->speed= speed;
    this->steerGain= steerGain;
}

dReal vmCar::getTotalMass()
{
    dReal mass= chassis.mass + frWheel.mass + flWheel.mass
            + rrWheel.mass + rlWheel.mass;
    return mass;
}

dReal vmCar::getNonlinearKd(vm::WheelLoc loc, dReal step)
{
    // select wheel
    vmWheel *wnow;
    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        break;
    default:
        break;
    }

    // compute vz
    dReal vz= getSuspensionRate(loc, step);


    // calculate kd
    dReal kd;
    if (vz<0.0)
        kd= 1000.0;
    else if (vz<250.0)
        kd= 4000.0;
    else
        kd= 1539.0;

    //printf("dz=%12.6f; vz=%12.6f; kd=%12.6f\n",
    //       dzNew, vz, kd);
    return kd;
}

dReal vmCar::getSuspensionRate(vm::WheelLoc loc, dReal step)
{
    // select wheel
    vmWheel *wnow= nullptr;
    switch (loc) {
    case vm::WheelLoc::FR:
        wnow= &frWheel;
        break;
    case vm::WheelLoc::FL:
        wnow= &flWheel;
        break;
    case vm::WheelLoc::RR:
        wnow= &rrWheel;
        break;
    case vm::WheelLoc::RL:
        wnow= &rlWheel;
        break;
    default:
        break;
    }

    dVector3 res1,res2;
    dJointGetHinge2Anchor(wnow->joint, res1);
    dJointGetHinge2Anchor2(wnow->joint, res2);
    dReal dzNew = res1[2]- res2[2];
    // update old dz
    wnow->dzSuspension= dzNew;

    // compute vz
    return (dzNew- wnow->dzSuspension)/step;
}


void vmCar::listVehiclePosition(std::ofstream &fp, dReal simCt, dReal step)
{
    const dReal *pos= dBodyGetPosition(chassis.body);

    const dReal *ep= dBodyGetQuaternion(chassis.body);
    dReal roll= ep[0]*ep[1];
    dReal pitch= ep[0]*ep[2];
    dReal yaw= ep[0]*ep[3];

    if (simCt==1)
    {
        fp<<"Time,"<<"PosX,"<<"PosY,"<<"PosZ,"<<"Roll,"<<"Pitch,"<<"Yaw\n";        
    }
	fp << simCt*step << "," << pos[0] << "," << pos[1] << "," << pos[2]
		<< "," << roll << "," << pitch << "," << yaw << "\n";
    
}

void vmCar::listWheelForce(std::ofstream &fp, dReal simCt, dReal step)
{

    dJointFeedback *fb0= frWheel.feedback;
    dJointFeedback *fb1= flWheel.feedback;
    dJointFeedback *fb2= rrWheel.feedback;
    dJointFeedback *fb3= rlWheel.feedback;

	if (simCt == 1)
	{
		fp << "Time," << "FRChss-Fx," << "FRChss-Fy," << "FRChss-Fz," <<
			"FLChss-Fx," << "FLChss-Fy," << "FLChss-Fz," <<
			"RRChss-Fx," << "RRChss-Fy," << "RRChss-Fz," <<
			"RLChss-Fx," << "RLChss-Fy," << "RLChss-Fz\n";
	}
    fp<<simCt*step<<","<<fb0->f1[0]<<","<<fb0->f1[1]<<","<<fb0->f1[2]<<","<<
                fb1->f1[0]<<","<<fb1->f1[1]<<","<<fb1->f1[2]<<","<<
                fb2->f1[0]<<","<<fb2->f1[1]<<","<<fb2->f1[2]<<","<<
                fb3->f1[0]<<","<<fb3->f1[1]<<","<<fb3->f1[2]<<"\n";
    
    
}

void vmCar::SetWheelJoint(vm::SIDE side, int idx, bool steerable)
{
    vmWheel *wnow= nullptr;
    
	if (side == vm::LEFT)
	{
		wnow = &left_wheels_[idx];
	}
	else
	{
		wnow = &right_wheels_[idx];
	}
	
    // create joint
    wnow->joint= dJointCreateHinge2(world_,0);
    // attach joint
    dJointAttach(wnow->joint,chassis_.body,wnow->body);
    // set joint
    const dReal *pos;
    pos= dBodyGetPosition(wnow->body);
    dJointSetHinge2Anchor(wnow->joint,pos[0],pos[1],pos[2]);
    dJointSetHinge2Axis1(wnow->joint,0.0, 0.0, 1.0); // steering
    dJointSetHinge2Axis2(wnow->joint,0.0, 1.0, 0.0); // rotation

    // lock non-steering wheel to keep it align to 0 degree (longitudinally)
    if (!steerable)
    {
        dJointSetHinge2Param(wnow->joint, dParamLoStop, 0.0);
        dJointSetHinge2Param(wnow->joint, dParamHiStop, 0.0);
    }

    // set joint feedback
    wnow->feedback= new dJointFeedback;
    dJointSetFeedback(wnow->joint,wnow->feedback);
}


void vmCar::SetWheelSuspension(vm::SIDE side, int idx, dReal step, dReal kps, dReal kds)
{
    vmWheel *wnow= nullptr;
	if (side == vm::LEFT)
	{
		wnow = &left_wheels_[idx];
	}
	else
	{
		wnow = &right_wheels_[idx];
	}    

    dJointSetHinge2Param(wnow->joint,dParamSuspensionERP, vm::ComputeERP(step,kps,kds)); // suspension
    dJointSetHinge2Param(wnow->joint,dParamSuspensionCFM, vm::ComputeCFM(step,kps,kds)); // suspension
}

void vmCar::SetAllWheelSuspension(dReal step, dReal kps, dReal kds)
{
	for (size_t i = 0; i < left_wheels_.size(); i++)
	{
		SetWheelSuspension(vm::LEFT, i, step, kps, kds);
		SetWheelSuspension(vm::RIGHT, i, step, kps, kds);
	}
}

void vmCar::SimCommand(int cmd)
{
    // manual command
	manual_act_ = true;
	brake_act_ = false;
    switch (cmd)
    {
    case 'i': case 'I':
        speed+= 0.25*M_PI;
        steer*= 0.98;
        break;
    case ',': case '<':
        speed*= 0.1*M_PI;
        brakeYes = 1;
        steer*= 0.98;
        break;
    case 'k': case 'K':
        speed-= 0.25*M_PI;
        steer*= 0.98;
        break;
    case 'j': case 'J':
        steer= -40*M_PI/180;
        break;
    case 'l': case 'L':
        steer= 40*M_PI/180;
        break;
    default:
        manual_act_ = false;
		brake_act_ = false;
        steer*= 0.98;
        speed*= 0.999;
    }

    //setInitialControls(steer,speed,steerGain);
}

void vmCar::SetChassis(dReal mass, dReal length, dReal width, dReal height)
{
	chassis_.SetProperty(world_, space_, mass, length, width, height);
}

/* set wheel properties
@side: left or right side
@idx: index of the axle
@mass: mass of the wheel
@length: width of the wheel cylinder
@radius: radius of the wheel cyliner
*/
void vmCar::SetWheel(vm::SIDE side, int idx, dReal mass, dReal length, dReal radius)
{
	vmWheel *wnow= nullptr;
	if (side == vm::LEFT)
	{
		wnow = &left_wheels_[idx];
	}
	else
	{
		wnow = &right_wheels_[idx];
	}
	wnow->SetProperty(world_, space_, mass, length, radius);    
}

/* set all wheels properties at once
@mass: mass of the wheel
@length: width of the wheel cylinder
@radius: radius of the wheel cyliner
*/
void vmCar::SetAllWheel(dReal mass, dReal length, dReal radius)
{
	// set up left side wheels
	for (size_t i = 0; i < left_wheels_.size(); i++)
	{
		SetWheel(vm::LEFT, i,mass, length, radius);
	}
	// set up right side wheels
	for (size_t i=0; i< right_wheels_.size();i++)
	{
		SetWheel(vm::RIGHT, i, mass, length, radius);
	}
}


// compute ERP and CFM
dReal vm::ComputeERP(dReal step, dReal kp, dReal kd)
{
    return step*kp/(step*kp + kd);
}
dReal vm::ComputeCFM(dReal step, dReal kp, dReal kd)
{
    return 1.0/(step*kp + kd);
}

//bounded function
dReal vm::Bounded(dReal var, dReal lb, dReal ub)
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

void vmWheel::SetProperty(const dWorldID &world, const dSpaceID &space, 
	dReal mass, dReal length, dReal radius)
{
	// setup now
	this->radius = radius;
	this->length = length;
	this->mass = mass;

	body = dBodyCreate(world);
	// setup geometry (assume body and geom centers are same for now)
	geom.geomEnc = dCreateCylinder(0, this->radius, this->length);
	geom.geomT = dCreateGeomTransform(space);
	dGeomTransformSetGeom(geom.geomT, geom.geomEnc);
	dGeomTransformSetInfo(geom.geomT, 1);

	// set mass (initially axial direction is parallel to global z)
	dMass m1;
	dMassSetZero(&m1);
	dMassSetCylinderTotal(&m1, this->mass, 3, this->radius, this->length);
	dBodySetMass(body, &m1);

	// set geom
	dGeomSetBody(geom.geomT, body);

	initialized = true;
}
