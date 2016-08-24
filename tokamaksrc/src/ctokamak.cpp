
#include "tokamak.h"

#define IMPLEMENTING_TOKAMAK_C_AS_CPP
#include "ctokamak.h"

#include <stddef.h> // for offsetof

#define BuildAssertSize(expr, size) \
  (void) sizeof(char[1 - 50 * ((size) - (expr))]); \
  (void) sizeof(char[1 - 50 * ((expr) - (size))]);

#define BuildAssertExactlySameSizeAndMembers(cType, cppType, firstMember, lastMember) \
  BuildAssertSize(offsetof(cppType, firstMember), offsetof(cType, firstMember)); \
  BuildAssertSize(offsetof(cppType, lastMember), offsetof(cType, lastMember)); \
  BuildAssertSize(sizeof(cppType), sizeof(cType));

#define RETURN_TOKAMAK_C_POINTER(cppThing, cType, cppType, firstMember, lastMember) \
  { \
    cType * cThing; \
    \
    BuildAssertSize( \
      (offsetof(cppType, lastMember) + sizeof((cppThing)->lastMember)) - offsetof(cppType, firstMember), \
      (offsetof(cType, lastMember) + sizeof((cThing)->lastMember)) - offsetof(cType, firstMember) \
    ); \
    BuildAssertSize(sizeof(char), 1); \
    \
    cThing = (cType*)( \
      ((char*)(cppThing)) \
      + offsetof(cppType, firstMember) \
      - offsetof(cType, firstMember) \
    ); \
    return cThing; \
  }

#define DELETE_TOKAMAK_C_POINTER(cThing, cType, cppType, firstMember, lastMember) \
  { \
    cppType * cppThing;\
    \
    BuildAssertSize( \
      (offsetof(cppType, lastMember) + sizeof((cppThing)->lastMember)) - offsetof(cppType, firstMember), \
      (offsetof(cType, lastMember) + sizeof((cThing)->lastMember)) - offsetof(cType, firstMember) \
    ); \
    BuildAssertSize(sizeof(char), 1); \
    \
    cppThing = (cppType*)( \
      ((char*)(cThing)) \
      + offsetof(cType, firstMember) \
      - offsetof(cppType, firstMember) \
    ); \
    \
    delete cppThing; \
  }

extern "C"
{
  cneSimulatorSizeInfo * cneSimulatorSizeInfo_Create()
  {
    neSimulatorSizeInfo * thing = new neSimulatorSizeInfo();
    RETURN_TOKAMAK_C_POINTER(thing, cneSimulatorSizeInfo, neSimulatorSizeInfo, rigidBodiesCount, terrainNodesGrowByCount);
  }

  void cneSimulatorSizeInfo_Destroy(cneSimulatorSizeInfo ** obj)
  {
    DELETE_TOKAMAK_C_POINTER(*obj, cneSimulatorSizeInfo, neSimulatorSizeInfo, rigidBodiesCount, terrainNodesGrowByCount);
    *obj = 0;
  }

  cneSimulator * cneSimulator_CreateSimulator(
    const cneSimulatorSizeInfo * sizeInfo, 
    cneAllocatorAbstract * alloc, // may be null
    const cneV3 * gravity) // may be null
  {
    BuildAssertSize(sizeof(cneSimulatorSizeInfo), sizeof(neSimulatorSizeInfo));
    BuildAssertSize(sizeof(cneV3), sizeof(neV3));
    // TODO: handle non-default allocator
    return (cneSimulator*)neSimulator::CreateSimulator(*((neSimulatorSizeInfo*)sizeInfo), 0, (neV3*)gravity);
  }

  cneRigidBody * cneSimulator_CreateRigidBody(cneSimulator * obj)
  {
    return (cneRigidBody*)((neSimulator*)obj)->CreateRigidBody();
  }

  cneAnimatedBody * cneSimulator_CreateAnimatedBody(cneSimulator * obj)
  {
    return (cneAnimatedBody*)((neSimulator*)obj)->CreateAnimatedBody();
  }

  void cneSimulator_DestroySimulator(cneSimulator * sim)
  {
    neSimulator::DestroySimulator((neSimulator*)sim);
  }

  void cneSimulator_Advance(cneSimulator * obj, f32 sec, s32 nSteps, cnePerformanceReport * perfReport)
  {
    // TODO: handle non-nil perfReport
    ((neSimulator*)obj)->Advance(sec, nSteps);
  }

  void cneV3_Init(cneV3 * obj)
  {
    memset(obj, 0, sizeof(cneV3));
  }

  void cneT3_Init(cneT3 * obj)
  {
    memset(obj, 0, sizeof(cneT3));
  }

  void cneV3_Set(cneV3 * obj, f32 x, f32 y, f32 z)
  {
    BuildAssertExactlySameSizeAndMembers(cneV3, neV3, v, v);
    (*((neV3*)obj)).Set(x, y, z);
  }

  cneGeometry * cneRigidBody_AddGeometry(cneRigidBody * obj)
  {
    return (cneGeometry*)((neRigidBody*)obj)->AddGeometry();
  }

  void cneRigidBody_UpdateBoundingInfo(cneRigidBody * obj)
  {
    ((neRigidBody*)obj)->UpdateBoundingInfo();
  }

  void cneRigidBody_SetInertiaTensorV3(cneRigidBody * obj, const cneV3 * tensor)
  {
    ((neRigidBody*)obj)->SetInertiaTensor(*((const neV3*)tensor));
  }

  void cneRigidBody_SetMass(cneRigidBody * obj, f32 mass)
  {
    ((neRigidBody*)obj)->SetMass(mass);
  }

  void cneRigidBody_SetPos(cneRigidBody * obj, const cneV3 * p)
  {
    ((neRigidBody*)obj)->SetPos(*((const neV3*)p));
  }

  void cneRigidBody_BeginIterateGeometry(cneRigidBody * obj)
  {
    ((neRigidBody*)obj)->BeginIterateGeometry();
  }

  cneT3 cneRigidBody_GetTransform(cneRigidBody * obj)
  {
    BuildAssertExactlySameSizeAndMembers(cneT3, neT3, rot, pos);
    neT3 t = ((neRigidBody*)obj)->GetTransform();
    return *(cneT3*)(&t);
  }
  
  cneGeometry * cneRigidBody_GetNextGeometry(cneRigidBody * obj)
  {
    return (cneGeometry*)((neRigidBody*)obj)->GetNextGeometry();
  }

  cneGeometry * cneAnimatedBody_AddGeometry(cneAnimatedBody * obj)
  {
    return (cneGeometry*)((neAnimatedBody*)obj)->AddGeometry();
  }

  void cneAnimatedBody_UpdateBoundingInfo(cneAnimatedBody * obj)
  {
    ((neAnimatedBody*)obj)->UpdateBoundingInfo();
  }

  void cneAnimatedBody_SetPos(cneAnimatedBody * obj, const cneV3 * p)
  {
    ((neAnimatedBody*)obj)->SetPos(*((const neV3*)p));
  }

  void cneGeometry_SetBoxSizeV3(cneGeometry * obj, const cneV3 * boxSize)
  {
    ((neGeometry*)obj)->SetBoxSize(*((const neV3*)boxSize));
  }

  cneV3 cneBoxInertiaTensorV3(const cneV3 * boxSize, f32 mass)
  {
    BuildAssertExactlySameSizeAndMembers(cneV3, neV3, v, v);
    neV3 result = neBoxInertiaTensor(*((const neV3*)boxSize), mass);
    return *((const cneV3*)&result);
  }
}
