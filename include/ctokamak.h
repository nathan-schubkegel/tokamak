#ifndef CTOKAMAK_H
#define CTOKAMAK_H

#ifdef IMPLEMENTING_TOKAMAK_C_AS_CPP
extern "C"
{
#endif


#include "math/cne_math.h"


TOKAMAK_STRUCT_DECL(cneRigidBody);




typedef int cneBodyType;
#define CNE_TERRAIN 0
#define CNE_RIGID_BODY 1
#define CNE_ANIMATED_BODY 2




TOKAMAK_STRUCT_DECL(cneAllocatorAbstract);
cneByte * cneAllocatorAbstract_Alloc(cneAllocatorAbstract * obj, s32 size, s32 alignment);
void cneAllocatorAbstract_Free(cneAllocatorAbstract * obj, cneByte * buffer);




TOKAMAK_STRUCT(cneAllocatorDefault)
{
  s32 usedMem;
};
cneAllocatorAbstract * cneAllocatorDefault_ToAbstract(cneAllocatorDefault * obj);
cneAllocatorDefault * cneAllocatorDefault_Create();
void cneAllocatorDefault_Destroy(cneAllocatorDefault ** obj);
cneByte * cneAllocatorDefault_Alloc(cneAllocatorDefault * obj, s32 size, s32 alignment);
void cneAllocatorDefault_Free(cneAllocatorDefault * obj, cneByte * buffer);




TOKAMAK_STRUCT(cnePerformanceReport)
{
#define CNE_PERF_TOTAL_TIME 0
#define CNE_PERF_DYNAMIC 1
#define CNE_PERF_POSITION 2
#define CNE_PERF_CONTRAIN_SOLVING_1 3
#define CNE_PERF_CONTRAIN_SOLVING_2 4
#define CNE_PERF_COLLISION_DETECTION 5
#define CNE_PERF_COLLISION_CULLING 6
#define CNE_PERF_TERRAIN_CULLING 7
#define CNE_PERF_TERRAIN 8
#define CNE_PERF_CONTROLLER_CALLBACK 9
#define CNE_PERF_LAST 10
#define CNE_PERF_RUNNING_AVERAGE 0
#define CNE_PERF_SAMPLE 1

  f32 time[CNE_PERF_LAST];
  f32 accTime[CNE_PERF_LAST];
  s32 reportType;
  s32 numSample;
};
cnePerformanceReport * cnePerformanceReport_Create();
void cnePerformanceReport_Destroy(cnePerformanceReport ** obj);
void cnePerformanceReport_Reset(cnePerformanceReport * obj);
void cnePerformanceReport_SetReportType(cnePerformanceReport * obj, s32 type);




typedef int cneBreakFlag;
  // FIXME: I'm not sure if these were supposed to start at 0 or 1
#define CNE_BREAK_DISABLE 0
#define CNE_BREAK_NORMAL 1
#define CNE_BREAK_ALL 2
#define CNE_BREAK_NEIGHBOUR 3
// the following are the same as above, 
// except they create a rigid particle instead of a rigid body 
#define CNE_BREAK_NORMAL_PARTICLE 4
#define CNE_BREAK_ALL_PARTICLE 5
#define CNE_BREAK_NEIGHBOUR_PARTICLE 6




TOKAMAK_STRUCT_DECL(cneGeometry);
void cneGeometry_SetTransform(cneGeometry * obj, cneT3 * t);
void cneGeometry_SetMaterialIndex(cneGeometry * obj, s32 index);
s32 cneGeometry_GetMaterialIndex(cneGeometry * obj);
cneT3 cneGeometry_GetTransform(cneGeometry * obj);
void cneGeometry_SetUserData(cneGeometry * obj, u32 userData);
u32 cneGeometry_GetUserData(cneGeometry * obj);
void cneGeometry_SetBoxSize(cneGeometry * obj, f32 width, f32 height, f32 depth);
void cneGeometry_SetBoxSizeV3(cneGeometry * obj, const cneV3 * boxSize);
cneBool cneGeometry_GetBoxSize(cneGeometry * obj, cneV3 * boxSize); // return false if geometry is not a box
void cneGeometry_SetSphereDiameter(cneGeometry * obj, f32 diameter);
cneBool cneGeometry_GetSphereDiameter(cneGeometry * obj, f32 * diameter); // return false if geometry is not a sphere
void cneGeometry_SetCylinder(cneGeometry * obj, f32 diameter, f32 height);
cneBool cneGeometry_GetCylinder(cneGeometry * obj, f32 * diameter, f32 * height); // return false if geometry is not a cylinder
void cneGeometry_SetConvexMesh(cneGeometry * obj, cneByte * convexData);
cneBool cneGeometry_GetConvexMesh(cneGeometry * obj, cneByte * convexData);
void cneGeometry_SetBreakageFlag(cneGeometry * obj, cneBreakFlag flag);
cneBreakFlag cneGeometry_GetBreakageFlag(cneGeometry * obj);
void cneGeometry_SetBreakageMass(cneGeometry * obj, f32 mass);
f32 cneGeometry_GetBreakageMass(cneGeometry * obj);
void cneGeometry_SetBreakageInertiaTensor(cneGeometry * obj, const cneV3 * tensor);
cneV3 cneGeometry_GetBreakageInertiaTensor(cneGeometry * obj);
void cneGeometry_SetBreakageMagnitude(cneGeometry * obj, f32 mag);
f32 cneGeometry_GetBreakageMagnitude(cneGeometry * obj);
void cneGeometry_SetBreakageAbsorption(cneGeometry * obj, f32 absorb);
f32 cneGeometry_GetBreakageAbsorption(cneGeometry * obj);
void cneGeometry_SetBreakagePlane(cneGeometry * obj, const cneV3 * planeNormal);
cneV3 cneGeometry_GetBreakagePlane(cneGeometry * obj);
void cneGeometry_SetBreakageNeighbourRadius(cneGeometry * obj, f32 radius);
f32 cneGeometry_GetBreakageNeighbourRadius(cneGeometry * obj);




typedef void (cneBreakageCallbackFunction)(void * userData, cneByte * originalBody, cneBodyType bodyType, cneGeometry * brokenGeometry, cneRigidBody * newBody);
typedef struct
{
  void * userData;
  cneBreakageCallbackFunction * function;
} cneBreakageCallback;




TOKAMAK_STRUCT(cneTriangle)
{
  s32 indices[3];
  s32 materialID;
  u32 flag;
  u32 userData;
};
cneTriangle * cneTriangle_Create();
void cneTriangle_Destroy(cneTriangle ** obj);
#define CNE_TRI_TRIANGLE 0
#define CNE_TRI_HEIGHT_MAP 1




TOKAMAK_STRUCT(cneTriangleMesh)
{
  // FIXME: need to confirm that sizeof(cneV3) == sizeof(neV3)
  cneV3 * vertices;
  s32 vertexCount;
  cneTriangle * triangles;
  s32 triangleCount;
};
cneTriangleMesh * cneTriangleMesh_Create();
void cneTriangleMesh_Destroy(cneTriangleMesh ** obj);




TOKAMAK_STRUCT_DECL(cneAnimatedBody);




TOKAMAK_STRUCT_DECL(cneSensor);
void SetLineSensor(cneSensor * obj, const cneV3 * pos, const cneV3 * lineVector);
void SetUserData(cneSensor * obj, u32 userData);
u32 GetUserData(cneSensor * obj);
// FIXME: can C return stuff bigger than 'int' ?
cneV3 GetLineVector(cneSensor * obj);
cneV3 GetLineUnitVector(cneSensor * obj);
cneV3 GetLinePos(cneSensor * obj);
f32 GetDetectDepth(cneSensor * obj);
cneV3 GetDetectNormal(cneSensor * obj);
cneV3 GetDetectContactPoint(cneSensor * obj);
cneRigidBody * GetDetectRigidBody(cneSensor * obj);
cneAnimatedBody * GetDetectAnimatedBody(cneSensor * obj);
s32 GetDetectMaterial(cneSensor * obj);




// cneAnimatedBody
// FIXME: can C return structs?
cneV3 cneAnimatedBody_GetPos(cneAnimatedBody * obj);
void cneAnimatedBody_SetPos(cneAnimatedBody * obj, const cneV3 * p);
cneM3 cneAnimatedBody_GetRotationM3(cneAnimatedBody * obj);
cneQ cneAnimatedBody_GetRotationQ(cneAnimatedBody * obj);
void cneAnimatedBody_SetRotation(cneAnimatedBody * obj, const cneM3 * m);
void cneAnimatedBody_SetRotationQ(cneAnimatedBody * obj, const cneQ * q);
cneT3 cneAnimatedBody_GetTransform(cneAnimatedBody * obj);
void cneAnimatedBody_SetCollisionID(cneAnimatedBody * obj, s32 cid);
s32 cneAnimatedBody_GetCollisionID(cneAnimatedBody * obj);
void cneAnimatedBody_SetUserData(cneAnimatedBody * obj, u32 userData);
u32 cneAnimatedBody_GetUserData(cneAnimatedBody * obj);
s32 cneAnimatedBody_GetGeometryCount(cneAnimatedBody * obj);
cneGeometry * cneAnimatedBody_AddGeometry(cneAnimatedBody * obj);
cneBool cneAnimatedBody_RemoveGeometry(cneAnimatedBody * obj, cneGeometry * g);
void cneAnimatedBody_BeginIterateGeometry(cneAnimatedBody * obj);
cneGeometry * cneAnimatedBody_GetNextGeometry(cneAnimatedBody * obj);
cneRigidBody * cneAnimatedBody_BreakGeometry(cneAnimatedBody * obj, cneGeometry * g);
cneSensor * cneAnimatedBody_AddSensor(cneAnimatedBody * obj);
cneBool cneAnimatedBody_RemoveSensor(cneAnimatedBody * obj, cneSensor * s);
void cneAnimatedBody_BeginIterateSensor(cneAnimatedBody * obj);
cneSensor * cneAnimatedBody_GetNextSensor(cneAnimatedBody * obj);
void cneAnimatedBody_UseCustomCollisionDetection(cneAnimatedBody * obj, cneBool yes, const cneT3 * obb, f32 boundingRadius);
cneBool cneAnimatedBody_GetUseCustomCollisionDetection(cneAnimatedBody * obj);
void cneAnimatedBody_UpdateBoundingInfo(cneAnimatedBody * obj);
void cneAnimatedBody_CollideConnected(cneAnimatedBody * obj, cneBool yes);
cneBool cneAnimatedBody_GetCollideConnected(cneAnimatedBody * obj);
void cneAnimatedBody_CollideDirectlyConnected(cneAnimatedBody * obj, cneBool yes);
cneBool cneAnimatedBody_GetCollideDirectlyConnected(cneAnimatedBody * obj);
void cneAnimatedBody_Active(cneAnimatedBody * obj, cneBool yes, cneRigidBody * hint); //  = NULL);
void cneAnimatedBody_ActiveA(cneAnimatedBody * obj, cneBool yes, cneAnimatedBody * hint); // = NULL);
cneBool cneAnimatedBody_GetActive(cneAnimatedBody * obj);




TOKAMAK_STRUCT_DECL(cneRigidBodyController);




TOKAMAK_STRUCT_DECL(cneJointController);




typedef void (cneRigidBodyControllerCallbackFunction)(void * userData, cneRigidBodyController * controller, float timeStep);
typedef struct
{
  void * userData;
  cneRigidBodyControllerCallbackFunction * function;
} cneRigidBodyControllerCallback;




typedef void (cneJointControllerCallbackFunction)(void * cuserData, cneJointController * controller, float timeStep);
typedef struct
{
  void * userData;
  cneJointControllerCallbackFunction * function;
} cneJointControllerCallback;



// cneRigidBody
f32 cneRigidBody_GetMass(cneRigidBody * obj);
void cneRigidBody_SetMass(cneRigidBody * obj, f32 mass);
void cneRigidBody_SetInertiaTensor(cneRigidBody * obj, const cneM3 * tensor);
void cneRigidBody_SetInertiaTensorV3(cneRigidBody * obj, const cneV3 * tensor);
void cneRigidBody_SetCollisionID(cneRigidBody * obj, s32 cid);
s32 cneRigidBody_GetCollisionID(cneRigidBody * obj);
void cneRigidBody_SetUserData(cneRigidBody * obj, u32 userData);
u32 cneRigidBody_GetUserData(cneRigidBody * obj);
s32 cneRigidBody_GetGeometryCount(cneRigidBody * obj);
void cneRigidBody_SetLinearDamping(cneRigidBody * obj, f32 damp);  
f32 cneRigidBody_GetLinearDamping(cneRigidBody * obj);
void cneRigidBody_SetAngularDamping(cneRigidBody * obj, f32 damp);  
f32 cneRigidBody_GetAngularDamping(cneRigidBody * obj);
void cneRigidBody_SetSleepingParameter(cneRigidBody * obj, f32 sleepingParam);
f32 cneRigidBody_GetSleepingParameter(cneRigidBody * obj);
cneGeometry * cneRigidBody_AddGeometry(cneRigidBody * obj);
cneBool cneRigidBody_RemoveGeometry(cneRigidBody * obj, cneGeometry * g);
void cneRigidBody_BeginIterateGeometry(cneRigidBody * obj);
cneGeometry * cneRigidBody_GetNextGeometry(cneRigidBody * obj);
cneRigidBody * cneRigidBody_BreakGeometry(cneRigidBody * obj, cneGeometry * g);
void cneRigidBody_UseCustomCollisionDetection(cneRigidBody * obj, cneBool yes, const cneT3 * obb, f32 boundingRadius);
cneBool cneRigidBody_GetUseCustomCollisionDetection(cneRigidBody * obj);
cneSensor * cneRigidBody_AddSensor(cneRigidBody * obj);
cneBool cneRigidBody_RemoveSensor(cneRigidBody * obj, cneSensor * s);
void cneRigidBody_BeginIterateSensor(cneRigidBody * obj);
cneSensor * cneRigidBody_GetNextSensor(cneRigidBody * obj);
cneRigidBodyController * cneRigidBody_AddController(cneRigidBody * obj, cneRigidBodyControllerCallback * controller, s32 period);
cneBool cneRigidBody_RemoveController(cneRigidBody * obj, cneRigidBodyController * rbController);
void cneRigidBody_BeginIterateController(cneRigidBody * obj);
cneRigidBodyController * cneRigidBody_GetNextController(cneRigidBody * obj);
cneV3 cneRigidBody_GetPos(cneRigidBody * obj);
void cneRigidBody_SetPos(cneRigidBody * obj, const cneV3 * p);
cneM3 cneRigidBody_GetRotationM3(cneRigidBody * obj);
cneQ cneRigidBody_GetRotationQ(cneRigidBody * obj);
void cneRigidBody_SetRotation(cneRigidBody * obj, const cneM3 * m);
void cneRigidBody_SetRotationQ(cneRigidBody * obj, const cneQ * q);
cneT3 cneRigidBody_GetTransform(cneRigidBody * obj);
cneV3 cneRigidBody_GetVelocity(cneRigidBody * obj);
void cneRigidBody_SetVelocity(cneRigidBody * obj, const cneV3 * v);
cneV3 cneRigidBody_GetAngularVelocity(cneRigidBody * obj);
cneV3 cneRigidBody_GetAngularMomentum(cneRigidBody * obj);
void cneRigidBody_SetAngularMomentum(cneRigidBody * obj, const cneV3 * am);
cneV3 cneRigidBody_GetVelocityAtPoint(cneRigidBody * obj, const cneV3 * pt);
void cneRigidBody_UpdateBoundingInfo(cneRigidBody * obj);
void cneRigidBody_UpdateInertiaTensor(cneRigidBody * obj);
void cneRigidBody_SetForce(cneRigidBody * obj, const cneV3 * force);
void cneRigidBody_SetTorque(cneRigidBody * obj, const cneV3 * torque);
void cneRigidBody_SetForce2(cneRigidBody * obj, const cneV3 * force, const cneV3 * pos);
cneV3 cneRigidBody_GetForce(cneRigidBody * obj);
cneV3 cneRigidBody_GetTorque(cneRigidBody * obj);
void cneRigidBody_ApplyImpulse(cneRigidBody * obj, const cneV3 * impulse);
void cneRigidBody_ApplyImpulse2(cneRigidBody * obj, const cneV3 * impulse, const cneV3 * pos);
void cneRigidBody_ApplyTwist(cneRigidBody * obj, const cneV3 * twist);
void cneRigidBody_GravityEnable(cneRigidBody * obj, cneBool yes);
cneBool cneRigidBody_GetGravityEnable(cneRigidBody * obj);
void cneRigidBody_CollideConnected(cneRigidBody * obj, cneBool yes); 
cneBool cneRigidBody_GetCollideConnected(cneRigidBody * obj);
void cneRigidBody_CollideDirectlyConnected(cneRigidBody * obj, cneBool yes);
cneBool cneRigidBody_GetCollideDirectlyConnected(cneRigidBody * obj);
void cneRigidBody_Active(cneRigidBody * obj, cneBool yes, cneRigidBody * hint); // = NULL);
void cneRigidBody_Active2(cneRigidBody * obj, cneBool yes, cneAnimatedBody * hint); // = NULL);
cneBool cneRigidBody_Active3(cneRigidBody * obj);
cneBool cneRigidBody_IsIdle(cneRigidBody * obj);




typedef int cneConstraintType;
// FIXME: should this start with 0 or 1?
#define CNE_JOINT_BALLSOCKET 0
#define CNE_JOINT_BALLSOCKET2 1
#define CNE_JOINT_HINGE 2
#define CNE_JOINT_SLIDE 3




// FIXME: 0 or 1?
typedef int cneMotorType;
#define CNE_MOTOR_SPEED 0
#define CNE_MOTOR_POSITION 1
// a comment said NE_MOTOR_POSITION is not implemented


TOKAMAK_STRUCT_DECL(cneJoint);
void cneJoint_SetType(cneJoint * obj, cneConstraintType t);
cneConstraintType cneJoint_GetType(cneJoint * obj);
void cneJoint_SetJointFrameA(cneJoint * obj, const cneT3 * frameA);
void cneJoint_SetJointFrameB(cneJoint * obj, const cneT3 * frameB);
void cneJoint_SetJointFrameWorld(cneJoint * obj, const cneT3 * frame);
cneT3 cneJoint_GetJointFrameA(cneJoint * obj);
cneT3 cneJoint_GetJointFrameB(cneJoint * obj);
void cneJoint_SetJointLength(cneJoint * obj, f32 length);
f32 cneJoint_GetJointLength(cneJoint * obj);
cneRigidBody * cneJoint_GetRigidBodyA(cneJoint * obj);
cneRigidBody * cneJoint_GetRigidBodyB(cneJoint * obj);
cneAnimatedBody * cneJoint_GetAnimatedBodyB(cneJoint * obj);
void cneJoint_Enable(cneJoint * obj, cneBool yes);
cneBool cneJoint_GetEnable(cneJoint * obj);
void cneJoint_SetDampingFactor(cneJoint * obj, f32 damp);
f32 cneJoint_GetDampingFactor(cneJoint * obj);
f32 cneJoint_GetPosition(cneJoint * obj);
f32 cneJoint_GetPosition2(cneJoint * obj);
cneBool cneJoint_GetEnableLimit(cneJoint * obj);
void cneJoint_EnableLimit(cneJoint * obj, cneBool yes);
f32 cneJoint_GetUpperLimit(cneJoint * obj);
void cneJoint_SetUpperLimit(cneJoint * obj, f32 upperLimit);
f32 cneJoint_GetLowerLimit(cneJoint * obj);
void cneJoint_SetLowerLimit(cneJoint * obj, f32 lowerLimit);
cneBool cneJoint_GetEnableLimit2(cneJoint * obj);
void cneJoint_EnableLimit2(cneJoint * obj, cneBool yes);
f32 cneJoint_GetUpperLimit2(cneJoint * obj);
void cneJoint_SetUpperLimit2(cneJoint * obj, f32 upperLimit);
f32 cneJoint_GetLowerLimit2(cneJoint * obj);
void cneJoint_SetLowerLimit2(cneJoint * obj, f32 lowerLimit);
void cneJoint_SetEpsilon(cneJoint * obj, f32 e);
f32 cneJoint_GetEpsilon(cneJoint * obj);
void cneJoint_SetIteration(cneJoint * obj, s32 i);
s32 cneJoint_GetIteration(cneJoint * obj);
cneJointController * cneJoint_AddController(cneJoint * obj, cneJointControllerCallback * controller, s32 period);
cneBool cneJoint_RemoveController(cneJoint * obj, cneJointController * rbController);
void cneJoint_BeginIterateController(cneJoint * obj);
cneJointController * cneJoint_GetNextController(cneJoint * obj);
cneBool cneJoint_GetEnableMotor(cneJoint * obj);
void cneJoint_EnableMotor(cneJoint * obj, cneBool yes);
void cneJoint_SetMotor(cneJoint * obj, cneMotorType motorType, f32 desireValue, f32 maxForce);
void cneJoint_GetMotor(cneJoint * obj, cneMotorType * motorType, f32 * desireValue, f32 * maxForce);
cneBool cneJoint_GetEnableMotor2(cneJoint * obj);
void cneJoint_EnableMotor2(cneJoint * obj, cneBool yes);
void cneJoint_SetMotor2(cneJoint * obj, cneMotorType motorType, f32 desireValue, f32 maxForce);
void cneJoint_GetMotor2(cneJoint * obj, cneMotorType * motorType, f32 * desireValue, f32 * maxForce);




// cneRigidBodyController
cneRigidBody * cneRigidBodyController_GetRigidBody(cneRigidBodyController * obj);
cneV3 cneRigidBodyController_GetControllerForce(cneRigidBodyController * obj);
cneV3 cneRigidBodyController_GetControllerTorque(cneRigidBodyController * obj);
void cneRigidBodyController_SetControllerForce(cneRigidBodyController * obj, const cneV3 * force);
void cneRigidBodyController_SetControllerTorque(cneRigidBodyController * obj, const cneV3 * torque);
void cneRigidBodyController_SetControllerForceWithTorque(cneRigidBodyController * obj, const cneV3 * force, const cneV3 * pos);




// cneJointController
cneJoint * cneJointController_GetJoint();
cneV3 cneJointController_GetControllerForceBodyA(cneJointController * obj);
cneV3 cneJointController_GetControllerForceBodyB(cneJointController * obj);
cneV3 cneJointController_GetControllerTorqueBodyA(cneJointController * obj);
cneV3 cneJointController_GetControllerTorqueBodyB(cneJointController * obj);
void cneJointController_SetControllerForceBodyA(cneJointController * obj, const cneV3 * force);
void cneJointController_SetControllerForceBodyB(cneJointController * obj, const cneV3 * force);
void cneJointController_SetControllerForceWithTorqueBodyA(cneJointController * obj, const cneV3 * force, const cneV3 * pos);
void cneJointController_SetControllerForceWithTorqueBodyB(cneJointController * obj, const cneV3 * force, const cneV3 * pos);
void cneJointController_SetControllerTorqueBodyA(cneJointController * obj, const cneV3 * torque);
void cneJointController_SetControllerTorqueBodyB(cneJointController * obj, const cneV3 * torque);




typedef int cneResponseBitFlag;
#define CNE_RESPONSE_IGNORE 0
#define CNE_RESPONSE_IMPULSE 1
#define CNE_RESPONSE_CALLBACK 2
#define CNE_RESPONSE_IMPULSE_CALLBACK 3
#define CNE_COLLISION_TABLE_MAX 64




TOKAMAK_STRUCT_DECL(cneCollisionTable);
void cneCollisionTable_Set(s32 collisionID1, s32 collisionID2, cneResponseBitFlag response);
cneResponseBitFlag cneCollisionTable_Get(s32 collisionID1, s32 collisionID2);
s32 cneCollisionTable_GetMaxCollisionID();




#define CNE_DEFAULT_RIGIDBODIES_COUNT 50
#define CNE_DEFAULT_ANIMATEDBODIES_COUNT 50 
#define CNE_DEFAULT_RIGIDPARTICLES_COUNT 50

#define CNE_DEFAULT_CONTROLLERS_COUNT 50
#define CNE_DEFAULT_OVERLAPPED_PAIRS_COUNT 1225

#define CNE_DEFAULT_GEOMETRIES_COUNT 50

#define CNE_DEFAULT_CONSTRAINTS_COUNT 100
#define CNE_DEFAULT_CONTRAINT_SETS_COUNT 100
#define CNE_DEFAULT_SOLVER_BUFFER_SIZE 2000
#define CNE_DEFAULT_SENSORS_COUNT 100

#define CNE_DEFAULT_TERRAIN_NODES_START_COUNT 200
#define CNE_DEFAULT_TERRAIN_NODES_GROWBY_COUNT -1




TOKAMAK_STRUCT(cneSimulatorSizeInfo)
{
  s32 rigidBodiesCount;    /* Number of rigid bodies in the simulation */
  s32 animatedBodiesCount;  /* Number of animated bodies in the simulation */
  s32 rigidParticleCount;    /* Number of rigid particles in the simulation */

  s32 controllersCount;    /* Number of controller instances in the simulation */
  
  s32 overlappedPairsCount;  /* Number of possible overlapping pairs.
                   This has the maximum value of (n x (n - 1)) / 2,
                   where n = rigidBodyCount + animatedBodyCount.
                   But in practice it rarely reach that high.
                   You can try to specify a smaller number to save memory.
                */
  s32 geometriesCount;    /* Number of collision geometries in the simulator*/


  s32 constraintsCount;    /* Number of joints in the simulation */
  s32 constraintSetsCount;  /* Number of joint Sets in the simulation */
  s32 constraintBufferSize;  /* Size of the buffer use to solve joints */
  s32 sensorsCount;

  s32 terrainNodesStartCount;  /* Number of nodes use to store terrain triangles */
  s32 terrainNodesGrowByCount;/* Grow by this size if run out of nodes */
};
cneSimulatorSizeInfo * cneSimulatorSizeInfo_Create();
void cneSimulatorSizeInfo_Destroy(cneSimulatorSizeInfo ** obj);




typedef struct cneCollisionInfo cneCollisionInfo;
struct cneCollisionInfo
{
  cneByte * bodyA;
  cneByte * bodyB;
  cneBodyType typeA;
  cneBodyType typeB;
  cneGeometry * geometryA;
  cneGeometry * geometryB;
  s32 materialIdA;
  s32 materialIdB;
  cneV3 bodyContactPointA;    // contact point A in body space of A
  cneV3 bodyContactPointB;    // contact point B in body space of B
  cneV3 worldContactPointA;  // contact point A in world space
  cneV3 worldContactPointB;  // contact point B in world space
  cneV3 relativeVelocity;
  cneV3 collisionNormal;
};




typedef void (cneLogOutputCallbackFunction)(void * userData, char * logString);
typedef struct
{
  void * userData;
  cneLogOutputCallbackFunction * function;
} cneLogOutputCallback;




typedef void (cneCollisionCallbackFunction)(void * userData, cneCollisionInfo * collisionInfo);
typedef struct
{
  void * userData;
  cneCollisionCallbackFunction * function;
} cneCollisionCallback;




typedef void (cneTerrainTriangleQueryCallbackFunction)(
  void * userData, 
  const cneV3 * minBound, 
  const cneV3 * maxBound, 
	s32 ** candidateTriangles,
	cneTriangle ** triangles,
	cneV3 ** vertices,
	s32 * candidateCount,
	s32 * triangleCount,
	cneRigidBody * body);
typedef struct
{
  void * userData;
  cneTerrainTriangleQueryCallbackFunction * function;
} cneTerrainTriangleQueryCallback;




typedef struct cneCustomCDInfo cneCustomCDInfo;
struct cneCustomCDInfo
{
  cneV3 collisionNormal;
  cneV3 worldContactPointA;
  cneV3 worldContactPointB;
  f32 penetrationDepth;
  s32 materialIdA;
  s32 materialIdB;
};




typedef cneBool (cneCustomCDRB2RBCallbackFunction)(void * userData, cneRigidBody * bodyA, cneRigidBody * bodyB, cneCustomCDInfo * cdInfo);
typedef struct
{
  void * userData;
  cneCustomCDRB2RBCallbackFunction * function;
} cneCustomCDRB2RBCallback;




typedef cneBool (cneCustomCDRB2ABCallbackFunction)(void * userData, cneRigidBody * bodyA, cneAnimatedBody * bodyB, cneCustomCDInfo * cdInfo);
typedef struct
{
  void * userData;
  cneCustomCDRB2ABCallbackFunction * function;
} cneCustomCDRB2ABCallback;




TOKAMAK_STRUCT_DECL(cneSimulator);
typedef int CNE_LOG_OUTPUT_LEVEL;
#define CNE_LOG_OUTPUT_LEVEL_NONE 0
#define CNE_LOG_OUTPUT_LEVEL_ONE 1
#define CNE_LOG_OUTPUT_LEVEL_FULL 2
cneSimulator * cneSimulator_CreateSimulator(
  const cneSimulatorSizeInfo * sizeInfo, 
  cneAllocatorAbstract * alloc, // may be null
  const cneV3 * gravity); // may be null
void cneSimulator_DestroySimulator(cneSimulator * sim);
cneRigidBody * cneSimulator_CreateRigidBody(cneSimulator * obj);
cneRigidBody * cneSimulator_CreateRigidParticle(cneSimulator * obj);
cneAnimatedBody * cneSimulator_CreateAnimatedBody(cneSimulator * obj);
void cneSimulator_FreeRigidBody(cneSimulator * obj, cneRigidBody * body);
void cneSimulator_FreeAnimatedBody(cneSimulator * obj, cneAnimatedBody * body);
cneCollisionTable * cneSimulator_GetCollisionTable(cneSimulator * obj);
cneBool cneSimulator_SetMaterial(cneSimulator * obj, s32 index, f32 friction, f32 restitution);
cneBool cneSimulator_GetMaterial(cneSimulator * obj, s32 index, f32 * friction, f32 * restitution);
void cneSimulator_Advance(cneSimulator * obj, f32 sec, s32 nSteps /* = 1 */, cnePerformanceReport * perfReport /* = NULL */);
void cneSimulator_Advance2(cneSimulator * obj, f32 sec, f32 minTimeStep, f32 maxTimeStep, cnePerformanceReport * perfReport /* = NULL*/);
void cneSimulator_SetTerrainMesh(cneSimulator * obj, cneTriangleMesh * tris);
void cneSimulator_FreeTerrainMesh(cneSimulator * obj);
cneJoint * cneSimulator_CreateJoint(cneSimulator * obj, cneRigidBody * bodyA);
cneJoint * cneSimulator_CreateJoint2(cneSimulator * obj, cneRigidBody * bodyA, cneRigidBody * bodyB);
cneJoint * cneSimulator_CreateJoint3(cneSimulator * obj, cneRigidBody * bodyA, cneAnimatedBody * bodyB);
void cneSimulator_FreeJoint(cneSimulator * obj, cneJoint * joint);
cneV3 cneSimulator_GetGravity(cneSimulator * obj);
void cneSimulator_Gravity(cneSimulator * obj, const cneV3 * gravity);
void cneSimulator_SetBreakageCallback(cneSimulator * obj, cneBreakageCallback * cb);
cneBreakageCallback * cneSimulator_GetBreakageCallback(cneSimulator * obj);
void cneSimulator_SetCollisionCallback(cneSimulator * obj, cneCollisionCallback * cb);
cneCollisionCallback * cneSimulator_GetCollisionCallback(cneSimulator * obj);
void cneSimulator_SetTerrainTriangleQueryCallback(cneSimulator * obj, cneTerrainTriangleQueryCallback * cb);
cneTerrainTriangleQueryCallback * cneSimulator_GetTerrainTriangleQueryCallback(cneSimulator * obj);
void cneSimulator_SetCustomCDRB2RBCallback(cneSimulator * obj, cneCustomCDRB2RBCallback * cb);
cneCustomCDRB2RBCallback * cneSimulator_GetCustomCDRB2RBCallback(cneSimulator * obj);
void cneSimulator_SetCustomCDRB2ABCallback(cneSimulator * obj, cneCustomCDRB2ABCallback * cb);
cneCustomCDRB2ABCallback * cneSimulator_GetCustomCDRB2ABCallback(cneSimulator * obj);
void cneSimulator_SetLogOutputCallback(cneSimulator * obj, cneLogOutputCallback * cb);
cneLogOutputCallback * cneSimulator_GetLogOutputCallback(cneSimulator * obj);
void cneSimulator_SetLogOutputLevel(cneSimulator * obj, CNE_LOG_OUTPUT_LEVEL lvl); //= CNE_LOG_OUTPUT_LEVEL_FULL);
cneSimulatorSizeInfo cneSimulator_GetCurrentSizeInfo(cneSimulator * obj);
cneSimulatorSizeInfo cneSimulator_GetStartSizeInfo(cneSimulator * obj);
void cneSimulator_GetMemoryAllocated(cneSimulator * obj, s32 * memoryAllocated);




cneV3 cneBoxInertiaTensor(f32 width, f32 height, f32 depth, f32 mass);
cneV3 cneBoxInertiaTensorV3(const cneV3 * boxSize, f32 mass);
cneV3 cneSphereInertiaTensor(f32 diameter, f32 mass);
cneV3 cneCylinderInertiaTensor(f32 diameter, f32 height, f32 mass);




#ifdef IMPLEMENTING_TOKAMAK_C_AS_CPP
} // extern "C"
#endif

#endif // CTOKAMAK_H