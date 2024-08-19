namespace Box2DBeef;
using System;

static
{
public function void* b2AllocFcn(uint32 size, int32 alignment);

public function void b2FreeFcn(void* mem);

[CLink]
public static extern void b2SetAllocator(b2AllocFcn allocFcn, b2FreeFcn freeFcn);

[CLink]
public static extern uint32 b2GetByteCount();

public function int32 b2AssertFcn(char8* condition, char8* fileName, int32 lineNumber);

[CLink]
public static extern void b2SetAssertFcn(b2AssertFcn assertFcn);

[CRepr]
public struct b2Version
{
    public int32 major;
    public int32 minor;
    public int32 revision;
}

[CLink]
public static extern b2Version b2GetVersion();

[CRepr]
public struct b2Timer
{
    public int64 start;
}

[CLink]
public static extern b2Timer b2CreateTimer();

[CLink]
public static extern int64 b2GetTicks(b2Timer* timer);

[CLink]
public static extern float b2GetMilliseconds(b2Timer* timer);

[CLink]
public static extern float b2GetMillisecondsAndReset(b2Timer* timer);

[CLink]
public static extern void b2SleepMilliseconds(int32 milliseconds);

[CLink]
public static extern void b2Yield();

[CRepr]
public struct b2Vec2
{
    public float x;
    public float y;
}

[CRepr]
public struct b2Rot
{
    public float c;
    public float s;
}

[CRepr]
public struct b2Transform
{
    public b2Vec2 p;
    public b2Rot q;
}

[CRepr]
public struct b2Mat22
{
    public b2Vec2 cx;
    public b2Vec2 cy;
}

[CRepr]
public struct b2AABB
{
    public b2Vec2 lowerBound;
    public b2Vec2 upperBound;
}

[CLink]
public static extern float b2MinFloat(float a, float b);

[CLink]
public static extern float b2MaxFloat(float a, float b);

[CLink]
public static extern float b2AbsFloat(float a);

[CLink]
public static extern float b2ClampFloat(float a, float lower, float upper);

[CLink]
public static extern int32 b2MinInt(int32 a, int32 b);

[CLink]
public static extern int32 b2MaxInt(int32 a, int32 b);

[CLink]
public static extern int32 b2AbsInt(int32 a);

[CLink]
public static extern int32 b2ClampInt(int32 a, int32 lower, int32 upper);

[CLink]
public static extern float b2Dot(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern float b2Cross(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2CrossVS(b2Vec2 v, float s);

[CLink]
public static extern b2Vec2 b2CrossSV(float s, b2Vec2 v);

[CLink]
public static extern b2Vec2 b2LeftPerp(b2Vec2 v);

[CLink]
public static extern b2Vec2 b2RightPerp(b2Vec2 v);

[CLink]
public static extern b2Vec2 b2Add(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2Sub(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2Neg(b2Vec2 a);

[CLink]
public static extern b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t);

[CLink]
public static extern b2Vec2 b2Mul(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2MulSV(float s, b2Vec2 v);

[CLink]
public static extern b2Vec2 b2MulAdd(b2Vec2 a, float s, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2MulSub(b2Vec2 a, float s, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2Abs(b2Vec2 a);

[CLink]
public static extern b2Vec2 b2Min(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2Max(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Vec2 b2Clamp(b2Vec2 v, b2Vec2 a, b2Vec2 b);

[CLink]
public static extern float b2Length(b2Vec2 v);

[CLink]
public static extern float b2LengthSquared(b2Vec2 v);

[CLink]
public static extern float b2Distance(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern float b2DistanceSquared(b2Vec2 a, b2Vec2 b);

[CLink]
public static extern b2Rot b2MakeRot(float angle);

[CLink]
public static extern b2Rot b2NormalizeRot(b2Rot q);

[CLink]
public static extern bool b2IsNormalized(b2Rot q);

[CLink]
public static extern b2Rot b2NLerp(b2Rot q1, b2Rot q2, float t);

[CLink]
public static extern b2Rot b2IntegrateRotation(b2Rot q1, float deltaAngle);

[CLink]
public static extern float b2ComputeAngularVelocity(b2Rot q1, b2Rot q2, float inv_h);

[CLink]
public static extern float b2Rot_GetAngle(b2Rot q);

[CLink]
public static extern b2Vec2 b2Rot_GetXAxis(b2Rot q);

[CLink]
public static extern b2Vec2 b2Rot_GetYAxis(b2Rot q);

[CLink]
public static extern b2Rot b2MulRot(b2Rot q, b2Rot r);

[CLink]
public static extern b2Rot b2InvMulRot(b2Rot q, b2Rot r);

[CLink]
public static extern float b2RelativeAngle(b2Rot b, b2Rot a);

[CLink]
public static extern float b2UnwindAngle(float angle);

[CLink]
public static extern b2Vec2 b2RotateVector(b2Rot q, b2Vec2 v);

[CLink]
public static extern b2Vec2 b2InvRotateVector(b2Rot q, b2Vec2 v);

[CLink]
public static extern b2Vec2 b2TransformPoint(b2Transform t, b2Vec2 p);

[CLink]
public static extern b2Vec2 b2InvTransformPoint(b2Transform t, b2Vec2 p);

[CLink]
public static extern b2Transform b2MulTransforms(b2Transform A, b2Transform B);

[CLink]
public static extern b2Transform b2InvMulTransforms(b2Transform A, b2Transform B);

[CLink]
public static extern b2Vec2 b2MulMV(b2Mat22 A, b2Vec2 v);

[CLink]
public static extern b2Mat22 b2GetInverse22(b2Mat22 A);

[CLink]
public static extern b2Vec2 b2Solve22(b2Mat22 A, b2Vec2 b);

[CLink]
public static extern bool b2AABB_Contains(b2AABB a, b2AABB b);

[CLink]
public static extern b2Vec2 b2AABB_Center(b2AABB a);

[CLink]
public static extern b2Vec2 b2AABB_Extents(b2AABB a);

[CLink]
public static extern b2AABB b2AABB_Union(b2AABB a, b2AABB b);

[CLink]
public static extern bool b2IsValid(float a);

[CLink]
public static extern bool b2Vec2_IsValid(b2Vec2 v);

[CLink]
public static extern bool b2Rot_IsValid(b2Rot q);

[CLink]
public static extern bool b2AABB_IsValid(b2AABB aabb);

[CLink]
public static extern b2Vec2 b2Normalize(b2Vec2 v);

[CLink]
public static extern b2Vec2 b2NormalizeChecked(b2Vec2 v);

[CLink]
public static extern b2Vec2 b2GetLengthAndNormalize(float* length, b2Vec2 v);

[CLink]
public static extern void b2SetLengthUnitsPerMeter(float lengthUnits);

[CLink]
public static extern float b2GetLengthUnitsPerMeter();

[CRepr]
public struct b2Circle
{
    public b2Vec2 center;
    public float radius;
}

[CRepr]
public struct b2Capsule
{
    public b2Vec2 center1;
    public b2Vec2 center2;
    public float radius;
}

[CRepr]
public struct b2DistanceCache
{
    public uint16 count;
    public uint8[3] indexA;
    public uint8[3] indexB;
}

[CRepr]
public struct b2Polygon
{
    public b2Vec2[8] vertices;
    public b2Vec2[8] normals;
    public b2Vec2 centroid;
    public float radius;
    public int32 count;
}

[CRepr]
public struct b2Segment
{
    public b2Vec2 point1;
    public b2Vec2 point2;
}

[CRepr]
public struct b2SmoothSegment
{
    public b2Vec2 ghost1;
    public b2Segment segment;
    public b2Vec2 ghost2;
    public int32 chainId;
}

[CRepr]
public struct b2Hull
{
    public b2Vec2[8] points;
    public int32 count;
}

[CRepr]
public struct b2RayCastInput
{
    public b2Vec2 origin;
    public b2Vec2 translation;
    public float maxFraction;
}

[CRepr]
public struct b2ShapeCastInput
{
    public b2Vec2[8] points;
    public int32 count;
    public float radius;
    public b2Vec2 translation;
    public float maxFraction;
}

[CRepr]
public struct b2CastOutput
{
    public b2Vec2 normal;
    public b2Vec2 point;
    public float fraction;
    public int32 iterations;
    public bool hit;
}

[CRepr]
public struct b2MassData
{
    public float mass;
    public b2Vec2 center;
    public float rotationalInertia;
}

[CLink]
public static extern bool b2IsValidRay(b2RayCastInput* input);

[CLink]
public static extern b2Polygon b2MakePolygon(b2Hull hull, float radius);

[CLink]
public static extern b2Polygon b2MakeOffsetPolygon(b2Hull hull, float radius, b2Transform transform);

[CLink]
public static extern b2Polygon b2MakeSquare(float h);

[CLink]
public static extern b2Polygon b2MakeBox(float hx, float hy);

[CLink]
public static extern b2Polygon b2MakeRoundedBox(float hx, float hy, float radius);

[CLink]
public static extern b2Polygon b2MakeOffsetBox(float hx, float hy, b2Vec2 center, float angle);

[CLink]
public static extern b2Polygon b2TransformPolygon(b2Transform transform, b2Polygon* polygon);

[CLink]
public static extern b2MassData b2ComputeCircleMass(b2Circle* shape, float density);

[CLink]
public static extern b2MassData b2ComputeCapsuleMass(b2Capsule* shape, float density);

[CLink]
public static extern b2MassData b2ComputePolygonMass(b2Polygon* shape, float density);

[CLink]
public static extern b2AABB b2ComputeCircleAABB(b2Circle* shape, b2Transform transform);

[CLink]
public static extern b2AABB b2ComputeCapsuleAABB(b2Capsule* shape, b2Transform transform);

[CLink]
public static extern b2AABB b2ComputePolygonAABB(b2Polygon* shape, b2Transform transform);

[CLink]
public static extern b2AABB b2ComputeSegmentAABB(b2Segment* shape, b2Transform transform);

[CLink]
public static extern bool b2PointInCircle(b2Vec2 point, b2Circle* shape);

[CLink]
public static extern bool b2PointInCapsule(b2Vec2 point, b2Capsule* shape);

[CLink]
public static extern bool b2PointInPolygon(b2Vec2 point, b2Polygon* shape);

[CLink]
public static extern b2CastOutput b2RayCastCircle(b2RayCastInput* input, b2Circle* shape);

[CLink]
public static extern b2CastOutput b2RayCastCapsule(b2RayCastInput* input, b2Capsule* shape);

[CLink]
public static extern b2CastOutput b2RayCastSegment(b2RayCastInput* input, b2Segment* shape, bool oneSided);

[CLink]
public static extern b2CastOutput b2RayCastPolygon(b2RayCastInput* input, b2Polygon* shape);

[CLink]
public static extern b2CastOutput b2ShapeCastCircle(b2ShapeCastInput* input, b2Circle* shape);

[CLink]
public static extern b2CastOutput b2ShapeCastCapsule(b2ShapeCastInput* input, b2Capsule* shape);

[CLink]
public static extern b2CastOutput b2ShapeCastSegment(b2ShapeCastInput* input, b2Segment* shape);

[CLink]
public static extern b2CastOutput b2ShapeCastPolygon(b2ShapeCastInput* input, b2Polygon* shape);

[CLink]
public static extern b2Hull b2ComputeHull(b2Vec2* points, int32 count);

[CLink]
public static extern bool b2ValidateHull(b2Hull* hull);

[CRepr]
public struct b2SegmentDistanceResult
{
    public b2Vec2 closest1;
    public b2Vec2 closest2;
    public float fraction1;
    public float fraction2;
    public float distanceSquared;
}

[CLink]
public static extern b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2);

[CRepr]
public struct b2DistanceProxy
{
    public b2Vec2[8] points;
    public int32 count;
    public float radius;
}

[CRepr]
public struct b2DistanceInput
{
    public b2DistanceProxy proxyA;
    public b2DistanceProxy proxyB;
    public b2Transform transformA;
    public b2Transform transformB;
    public bool useRadii;
}

[CRepr]
public struct b2DistanceOutput
{
    public b2Vec2 pointA;
    public b2Vec2 pointB;
    public float distance;
    public int32 iterations;
    public int32 simplexCount;
}

[CRepr]
public struct b2SimplexVertex
{
    public b2Vec2 wA;
    public b2Vec2 wB;
    public b2Vec2 w;
    public float a;
    public int32 indexA;
    public int32 indexB;
}

[CRepr]
public struct b2Simplex
{
    public b2SimplexVertex v1;
    public b2SimplexVertex v2;
    public b2SimplexVertex v3;
    public int32 count;
}

[CLink]
public static extern b2DistanceOutput b2ShapeDistance(b2DistanceCache* cache, b2DistanceInput* input, b2Simplex* simplexes, int32 simplexCapacity);

[CRepr]
public struct b2ShapeCastPairInput
{
    public b2DistanceProxy proxyA;
    public b2DistanceProxy proxyB;
    public b2Transform transformA;
    public b2Transform transformB;
    public b2Vec2 translationB;
    public float maxFraction;
}

[CLink]
public static extern b2CastOutput b2ShapeCast(b2ShapeCastPairInput* input);

[CLink]
public static extern b2DistanceProxy b2MakeProxy(b2Vec2* vertices, int32 count, float radius);

[CRepr]
public struct b2Sweep
{
    public b2Vec2 localCenter;
    public b2Vec2 c1;
    public b2Vec2 c2;
    public b2Rot q1;
    public b2Rot q2;
}

[CLink]
public static extern b2Transform b2GetSweepTransform(b2Sweep* sweep, float time);

[CRepr]
public struct b2TOIInput
{
    public b2DistanceProxy proxyA;
    public b2DistanceProxy proxyB;
    public b2Sweep sweepA;
    public b2Sweep sweepB;
    public float tMax;
}

[CRepr, AllowDuplicates]
public enum b2TOIState : int32
{
    b2_toiStateUnknown = 0,
    b2_toiStateFailed = 1,
    b2_toiStateOverlapped = 2,
    b2_toiStateHit = 3,
    b2_toiStateSeparated = 4
}

[CRepr]
public struct b2TOIOutput
{
    public b2TOIState state;
    public float t;
}

[CLink]
public static extern b2TOIOutput b2TimeOfImpact(b2TOIInput* input);

[CRepr]
public struct b2ManifoldPoint
{
    public b2Vec2 point;
    public b2Vec2 anchorA;
    public b2Vec2 anchorB;
    public float separation;
    public float normalImpulse;
    public float tangentImpulse;
    public float maxNormalImpulse;
    public float normalVelocity;
    public uint16 id;
    public bool persisted;
}

[CRepr]
public struct b2Manifold
{
    public b2ManifoldPoint[2] points;
    public b2Vec2 normal;
    public int32 pointCount;
}

[CLink]
public static extern b2Manifold b2CollideCircles(b2Circle* circleA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideCapsuleAndCircle(b2Capsule* capsuleA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideSegmentAndCircle(b2Segment* segmentA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollidePolygonAndCircle(b2Polygon* polygonA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideCapsules(b2Capsule* capsuleA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideSegmentAndCapsule(b2Segment* segmentA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollidePolygonAndCapsule(b2Polygon* polygonA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollidePolygons(b2Polygon* polygonA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideSegmentAndPolygon(b2Segment* segmentA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideSmoothSegmentAndCircle(b2SmoothSegment* smoothSegmentA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

[CLink]
public static extern b2Manifold b2CollideSmoothSegmentAndCapsule(b2SmoothSegment* smoothSegmentA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB, b2DistanceCache* cache);

[CLink]
public static extern b2Manifold b2CollideSmoothSegmentAndPolygon(b2SmoothSegment* smoothSegmentA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB, b2DistanceCache* cache);

[CRepr]
public struct b2TreeNode
{
    public b2AABB aabb;
    public uint32 categoryBits;
    public int32 parent;
    public int32 next;
    public int32 child1;
    public int32 child2;
    public int32 userData;
    public int16 height;
    public bool enlarged;
    public uint8[9] pad;
}

[CRepr]
public struct b2DynamicTree
{
    public b2TreeNode* nodes;
    public int32 root;
    public int32 nodeCount;
    public int32 nodeCapacity;
    public int32 freeList;
    public int32 proxyCount;
    public int32* leafIndices;
    public b2AABB* leafBoxes;
    public b2Vec2* leafCenters;
    public int32* binIndices;
    public int32 rebuildCapacity;
}

[CLink]
public static extern b2DynamicTree b2DynamicTree_Create();

[CLink]
public static extern void b2DynamicTree_Destroy(b2DynamicTree* tree);

[CLink]
public static extern int32 b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, uint32 categoryBits, int32 userData);

[CLink]
public static extern void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int32 proxyId);

[CLink]
public static extern void b2DynamicTree_MoveProxy(b2DynamicTree* tree, int32 proxyId, b2AABB aabb);

[CLink]
public static extern void b2DynamicTree_EnlargeProxy(b2DynamicTree* tree, int32 proxyId, b2AABB aabb);

public function bool b2TreeQueryCallbackFcn(int32 proxyId, int32 userData, void* context);

[CLink]
public static extern void b2DynamicTree_Query(b2DynamicTree* tree, b2AABB aabb, uint32 maskBits, b2TreeQueryCallbackFcn callback, void* context);

public function float b2TreeRayCastCallbackFcn(b2RayCastInput* input, int32 proxyId, int32 userData, void* context);

[CLink]
public static extern void b2DynamicTree_RayCast(b2DynamicTree* tree, b2RayCastInput* input, uint32 maskBits, b2TreeRayCastCallbackFcn callback, void* context);

public function float b2TreeShapeCastCallbackFcn(b2ShapeCastInput* input, int32 proxyId, int32 userData, void* context);

[CLink]
public static extern void b2DynamicTree_ShapeCast(b2DynamicTree* tree, b2ShapeCastInput* input, uint32 maskBits, b2TreeShapeCastCallbackFcn callback, void* context);

[CLink]
public static extern void b2DynamicTree_Validate(b2DynamicTree* tree);

[CLink]
public static extern int32 b2DynamicTree_GetHeight(b2DynamicTree* tree);

[CLink]
public static extern int32 b2DynamicTree_GetMaxBalance(b2DynamicTree* tree);

[CLink]
public static extern float b2DynamicTree_GetAreaRatio(b2DynamicTree* tree);

[CLink]
public static extern void b2DynamicTree_RebuildBottomUp(b2DynamicTree* tree);

[CLink]
public static extern int32 b2DynamicTree_GetProxyCount(b2DynamicTree* tree);

[CLink]
public static extern int32 b2DynamicTree_Rebuild(b2DynamicTree* tree, bool fullBuild);

[CLink]
public static extern void b2DynamicTree_ShiftOrigin(b2DynamicTree* tree, b2Vec2 newOrigin);

[CLink]
public static extern int32 b2DynamicTree_GetByteCount(b2DynamicTree* tree);

[CLink]
public static extern int32 b2DynamicTree_GetUserData(b2DynamicTree* tree, int32 proxyId);

[CLink]
public static extern b2AABB b2DynamicTree_GetAABB(b2DynamicTree* tree, int32 proxyId);

[CRepr]
public struct b2WorldId
{
    public uint16 index1;
    public uint16 revision;
}

[CRepr]
public struct b2BodyId
{
    public int32 index1;
    public uint16 world0;
    public uint16 revision;
}

[CRepr]
public struct b2ShapeId
{
    public int32 index1;
    public uint16 world0;
    public uint16 revision;
}

[CRepr]
public struct b2JointId
{
    public int32 index1;
    public uint16 world0;
    public uint16 revision;
}

[CRepr]
public struct b2ChainId
{
    public int32 index1;
    public uint16 world0;
    public uint16 revision;
}

public function void b2TaskCallback(int32 startIndex, int32 endIndex, uint32 workerIndex, void* taskContext);

public function void* b2EnqueueTaskCallback(b2TaskCallback* task, int32 itemCount, int32 minRange, void* taskContext, void* userContext);

public function void b2FinishTaskCallback(void* userTask, void* userContext);

[CRepr]
public struct b2RayResult
{
    public b2ShapeId shapeId;
    public b2Vec2 point;
    public b2Vec2 normal;
    public float fraction;
    public bool hit;
}

[CRepr]
public struct b2WorldDef
{
    public b2Vec2 gravity;
    public float restitutionThreshold;
    public float contactPushoutVelocity;
    public float hitEventThreshold;
    public float contactHertz;
    public float contactDampingRatio;
    public float jointHertz;
    public float jointDampingRatio;
    public bool enableSleep;
    public bool enableContinous;
    public int32 workerCount;
    public b2EnqueueTaskCallback enqueueTask;
    public b2FinishTaskCallback finishTask;
    public void* userTaskContext;
    public int32 internalValue;
}

[CLink]
public static extern b2WorldDef b2DefaultWorldDef();

[CRepr, AllowDuplicates]
public enum b2BodyType : int32
{
    b2_staticBody = 0,
    b2_kinematicBody = 1,
    b2_dynamicBody = 2,
    b2_bodyTypeCount = 3
}

[CRepr]
public struct b2BodyDef
{
    public b2BodyType type;
    public b2Vec2 position;
    public b2Rot rotation;
    public b2Vec2 linearVelocity;
    public float angularVelocity;
    public float linearDamping;
    public float angularDamping;
    public float gravityScale;
    public float sleepThreshold;
    public void* userData;
    public bool enableSleep;
    public bool isAwake;
    public bool fixedRotation;
    public bool isBullet;
    public bool isEnabled;
    public bool automaticMass;
    public int32 internalValue;
}

[CLink]
public static extern b2BodyDef b2DefaultBodyDef();

[CRepr]
public struct b2Filter
{
    public uint32 categoryBits;
    public uint32 maskBits;
    public int32 groupIndex;
}

[CLink]
public static extern b2Filter b2DefaultFilter();

[CRepr]
public struct b2QueryFilter
{
    public uint32 categoryBits;
    public uint32 maskBits;
}

[CLink]
public static extern b2QueryFilter b2DefaultQueryFilter();

[CRepr, AllowDuplicates]
public enum b2ShapeType : int32
{
    b2_circleShape = 0,
    b2_capsuleShape = 1,
    b2_segmentShape = 2,
    b2_polygonShape = 3,
    b2_smoothSegmentShape = 4,
    b2_shapeTypeCount = 5
}

[CRepr]
public struct b2ShapeDef
{
    public void* userData;
    public float friction;
    public float restitution;
    public float density;
    public b2Filter filter;
    public uint32 customColor;
    public bool isSensor;
    public bool enableSensorEvents;
    public bool enableContactEvents;
    public bool enableHitEvents;
    public bool enablePreSolveEvents;
    public bool forceContactCreation;
    public int32 internalValue;
}

[CLink]
public static extern b2ShapeDef b2DefaultShapeDef();

[CRepr]
public struct b2ChainDef
{
    public void* userData;
    public b2Vec2* points;
    public int32 count;
    public float friction;
    public float restitution;
    public b2Filter filter;
    public bool isLoop;
    public int32 internalValue;
}

[CLink]
public static extern b2ChainDef b2DefaultChainDef();

[CRepr]
public struct b2Profile
{
    public float step;
    public float pairs;
    public float collide;
    public float solve;
    public float buildIslands;
    public float solveConstraints;
    public float prepareTasks;
    public float solverTasks;
    public float prepareConstraints;
    public float integrateVelocities;
    public float warmStart;
    public float solveVelocities;
    public float integratePositions;
    public float relaxVelocities;
    public float applyRestitution;
    public float storeImpulses;
    public float finalizeBodies;
    public float splitIslands;
    public float sleepIslands;
    public float hitEvents;
    public float broadphase;
    public float continuous;
}

[CRepr]
public struct b2Counters
{
    public int32 staticBodyCount;
    public int32 bodyCount;
    public int32 shapeCount;
    public int32 contactCount;
    public int32 jointCount;
    public int32 islandCount;
    public int32 stackUsed;
    public int32 staticTreeHeight;
    public int32 treeHeight;
    public int32 byteCount;
    public int32 taskCount;
    public int32[12] colorCounts;
}

[CRepr, AllowDuplicates]
public enum b2JointType : int32
{
    b2_distanceJoint = 0,
    b2_motorJoint = 1,
    b2_mouseJoint = 2,
    b2_prismaticJoint = 3,
    b2_revoluteJoint = 4,
    b2_weldJoint = 5,
    b2_wheelJoint = 6
}

[CRepr]
public struct b2DistanceJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 localAnchorA;
    public b2Vec2 localAnchorB;
    public float length;
    public bool enableSpring;
    public float hertz;
    public float dampingRatio;
    public bool enableLimit;
    public float minLength;
    public float maxLength;
    public bool enableMotor;
    public float maxMotorForce;
    public float motorSpeed;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2DistanceJointDef b2DefaultDistanceJointDef();

[CRepr]
public struct b2MotorJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 linearOffset;
    public float angularOffset;
    public float maxForce;
    public float maxTorque;
    public float correctionFactor;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2MotorJointDef b2DefaultMotorJointDef();

[CRepr]
public struct b2MouseJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 target;
    public float hertz;
    public float dampingRatio;
    public float maxForce;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2MouseJointDef b2DefaultMouseJointDef();

[CRepr]
public struct b2PrismaticJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 localAnchorA;
    public b2Vec2 localAnchorB;
    public b2Vec2 localAxisA;
    public float referenceAngle;
    public bool enableSpring;
    public float hertz;
    public float dampingRatio;
    public bool enableLimit;
    public float lowerTranslation;
    public float upperTranslation;
    public bool enableMotor;
    public float maxMotorForce;
    public float motorSpeed;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2PrismaticJointDef b2DefaultPrismaticJointDef();

[CRepr]
public struct b2RevoluteJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 localAnchorA;
    public b2Vec2 localAnchorB;
    public float referenceAngle;
    public bool enableSpring;
    public float hertz;
    public float dampingRatio;
    public bool enableLimit;
    public float lowerAngle;
    public float upperAngle;
    public bool enableMotor;
    public float maxMotorTorque;
    public float motorSpeed;
    public float drawSize;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2RevoluteJointDef b2DefaultRevoluteJointDef();

[CRepr]
public struct b2WeldJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 localAnchorA;
    public b2Vec2 localAnchorB;
    public float referenceAngle;
    public float linearHertz;
    public float angularHertz;
    public float linearDampingRatio;
    public float angularDampingRatio;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2WeldJointDef b2DefaultWeldJointDef();

[CRepr]
public struct b2WheelJointDef
{
    public b2BodyId bodyIdA;
    public b2BodyId bodyIdB;
    public b2Vec2 localAnchorA;
    public b2Vec2 localAnchorB;
    public b2Vec2 localAxisA;
    public bool enableSpring;
    public float hertz;
    public float dampingRatio;
    public bool enableLimit;
    public float lowerTranslation;
    public float upperTranslation;
    public bool enableMotor;
    public float maxMotorTorque;
    public float motorSpeed;
    public bool collideConnected;
    public void* userData;
    public int32 internalValue;
}

[CLink]
public static extern b2WheelJointDef b2DefaultWheelJointDef();

[CRepr]
public struct b2SensorBeginTouchEvent
{
    public b2ShapeId sensorShapeId;
    public b2ShapeId visitorShapeId;
}

[CRepr]
public struct b2SensorEndTouchEvent
{
    public b2ShapeId sensorShapeId;
    public b2ShapeId visitorShapeId;
}

[CRepr]
public struct b2SensorEvents
{
    public b2SensorBeginTouchEvent* beginEvents;
    public b2SensorEndTouchEvent* endEvents;
    public int32 beginCount;
    public int32 endCount;
}

[CRepr]
public struct b2ContactBeginTouchEvent
{
    public b2ShapeId shapeIdA;
    public b2ShapeId shapeIdB;
}

[CRepr]
public struct b2ContactEndTouchEvent
{
    public b2ShapeId shapeIdA;
    public b2ShapeId shapeIdB;
}

[CRepr]
public struct b2ContactHitEvent
{
    public b2ShapeId shapeIdA;
    public b2ShapeId shapeIdB;
    public b2Vec2 point;
    public b2Vec2 normal;
    public float approachSpeed;
}

[CRepr]
public struct b2ContactEvents
{
    public b2ContactBeginTouchEvent* beginEvents;
    public b2ContactEndTouchEvent* endEvents;
    public b2ContactHitEvent* hitEvents;
    public int32 beginCount;
    public int32 endCount;
    public int32 hitCount;
}

[CRepr]
public struct b2BodyMoveEvent
{
    public b2Transform transform;
    public b2BodyId bodyId;
    public void* userData;
    public bool fellAsleep;
}

[CRepr]
public struct b2BodyEvents
{
    public b2BodyMoveEvent* moveEvents;
    public int32 moveCount;
}

[CRepr]
public struct b2ContactData
{
    public b2ShapeId shapeIdA;
    public b2ShapeId shapeIdB;
    public b2Manifold manifold;
}

public function bool b2CustomFilterFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context);

public function bool b2PreSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context);

public function bool b2OverlapResultFcn(b2ShapeId shapeId, void* context);

public function float b2CastResultFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context);

[CRepr, AllowDuplicates]
public enum b2HexColor : int32
{
    b2_colorAliceBlue = 15792383,
    b2_colorAntiqueWhite = 16444375,
    b2_colorAqua = 65535,
    b2_colorAquamarine = 8388564,
    b2_colorAzure = 15794175,
    b2_colorBeige = 16119260,
    b2_colorBisque = 16770244,
    b2_colorBlack = 0,
    b2_colorBlanchedAlmond = 16772045,
    b2_colorBlue = 255,
    b2_colorBlueViolet = 9055202,
    b2_colorBrown = 10824234,
    b2_colorBurlywood = 14596231,
    b2_colorCadetBlue = 6266528,
    b2_colorChartreuse = 8388352,
    b2_colorChocolate = 13789470,
    b2_colorCoral = 16744272,
    b2_colorCornflowerBlue = 6591981,
    b2_colorCornsilk = 16775388,
    b2_colorCrimson = 14423100,
    b2_colorCyan = 65535,
    b2_colorDarkBlue = 139,
    b2_colorDarkCyan = 35723,
    b2_colorDarkGoldenrod = 12092939,
    b2_colorDarkGray = 11119017,
    b2_colorDarkGreen = 25600,
    b2_colorDarkKhaki = 12433259,
    b2_colorDarkMagenta = 9109643,
    b2_colorDarkOliveGreen = 5597999,
    b2_colorDarkOrange = 16747520,
    b2_colorDarkOrchid = 10040012,
    b2_colorDarkRed = 9109504,
    b2_colorDarkSalmon = 15308410,
    b2_colorDarkSeaGreen = 9419919,
    b2_colorDarkSlateBlue = 4734347,
    b2_colorDarkSlateGray = 3100495,
    b2_colorDarkTurquoise = 52945,
    b2_colorDarkViolet = 9699539,
    b2_colorDeepPink = 16716947,
    b2_colorDeepSkyBlue = 49151,
    b2_colorDimGray = 6908265,
    b2_colorDodgerBlue = 2003199,
    b2_colorFirebrick = 11674146,
    b2_colorFloralWhite = 16775920,
    b2_colorForestGreen = 2263842,
    b2_colorFuchsia = 16711935,
    b2_colorGainsboro = 14474460,
    b2_colorGhostWhite = 16316671,
    b2_colorGold = 16766720,
    b2_colorGoldenrod = 14329120,
    b2_colorGray = 12500670,
    b2_colorGray1 = 1710618,
    b2_colorGray2 = 3355443,
    b2_colorGray3 = 5066061,
    b2_colorGray4 = 6710886,
    b2_colorGray5 = 8355711,
    b2_colorGray6 = 10066329,
    b2_colorGray7 = 11776947,
    b2_colorGray8 = 13421772,
    b2_colorGray9 = 15066597,
    b2_colorGreen = 65280,
    b2_colorGreenYellow = 11403055,
    b2_colorHoneydew = 15794160,
    b2_colorHotPink = 16738740,
    b2_colorIndianRed = 13458524,
    b2_colorIndigo = 4915330,
    b2_colorIvory = 16777200,
    b2_colorKhaki = 15787660,
    b2_colorLavender = 15132410,
    b2_colorLavenderBlush = 16773365,
    b2_colorLawnGreen = 8190976,
    b2_colorLemonChiffon = 16775885,
    b2_colorLightBlue = 11393254,
    b2_colorLightCoral = 15761536,
    b2_colorLightCyan = 14745599,
    b2_colorLightGoldenrod = 15654274,
    b2_colorLightGoldenrodYellow = 16448210,
    b2_colorLightGray = 13882323,
    b2_colorLightGreen = 9498256,
    b2_colorLightPink = 16758465,
    b2_colorLightSalmon = 16752762,
    b2_colorLightSeaGreen = 2142890,
    b2_colorLightSkyBlue = 8900346,
    b2_colorLightSlateBlue = 8679679,
    b2_colorLightSlateGray = 7833753,
    b2_colorLightSteelBlue = 11584734,
    b2_colorLightYellow = 16777184,
    b2_colorLime = 65280,
    b2_colorLimeGreen = 3329330,
    b2_colorLinen = 16445670,
    b2_colorMagenta = 16711935,
    b2_colorMaroon = 11546720,
    b2_colorMediumAquamarine = 6737322,
    b2_colorMediumBlue = 205,
    b2_colorMediumOrchid = 12211667,
    b2_colorMediumPurple = 9662683,
    b2_colorMediumSeaGreen = 3978097,
    b2_colorMediumSlateBlue = 8087790,
    b2_colorMediumSpringGreen = 64154,
    b2_colorMediumTurquoise = 4772300,
    b2_colorMediumVioletRed = 13047173,
    b2_colorMidnightBlue = 1644912,
    b2_colorMintCream = 16121850,
    b2_colorMistyRose = 16770273,
    b2_colorMoccasin = 16770229,
    b2_colorNavajoWhite = 16768685,
    b2_colorNavy = 128,
    b2_colorNavyBlue = 128,
    b2_colorOldLace = 16643558,
    b2_colorOlive = 8421376,
    b2_colorOliveDrab = 7048739,
    b2_colorOrange = 16753920,
    b2_colorOrangeRed = 16729344,
    b2_colorOrchid = 14315734,
    b2_colorPaleGoldenrod = 15657130,
    b2_colorPaleGreen = 10025880,
    b2_colorPaleTurquoise = 11529966,
    b2_colorPaleVioletRed = 14381203,
    b2_colorPapayaWhip = 16773077,
    b2_colorPeachPuff = 16767673,
    b2_colorPeru = 13468991,
    b2_colorPink = 16761035,
    b2_colorPlum = 14524637,
    b2_colorPowderBlue = 11591910,
    b2_colorPurple = 10494192,
    b2_colorRebeccaPurple = 6697881,
    b2_colorRed = 16711680,
    b2_colorRosyBrown = 12357519,
    b2_colorRoyalBlue = 4286945,
    b2_colorSaddleBrown = 9127187,
    b2_colorSalmon = 16416882,
    b2_colorSandyBrown = 16032864,
    b2_colorSeaGreen = 3050327,
    b2_colorSeashell = 16774638,
    b2_colorSienna = 10506797,
    b2_colorSilver = 12632256,
    b2_colorSkyBlue = 8900331,
    b2_colorSlateBlue = 6970061,
    b2_colorSlateGray = 7372944,
    b2_colorSnow = 16775930,
    b2_colorSpringGreen = 65407,
    b2_colorSteelBlue = 4620980,
    b2_colorTan = 13808780,
    b2_colorTeal = 32896,
    b2_colorThistle = 14204888,
    b2_colorTomato = 16737095,
    b2_colorTurquoise = 4251856,
    b2_colorViolet = 15631086,
    b2_colorVioletRed = 13639824,
    b2_colorWheat = 16113331,
    b2_colorWhite = 16777215,
    b2_colorWhiteSmoke = 16119285,
    b2_colorYellow = 16776960,
    b2_colorYellowGreen = 10145074,
    b2_colorBox2DRed = 14430514,
    b2_colorBox2DBlue = 3190463,
    b2_colorBox2DGreen = 9226532,
    b2_colorBox2DYellow = 16772748
}

[CRepr]
public struct b2DebugDraw
{
    public function void(b2Vec2* vertices, int32 vertexCount, b2HexColor color, void* context) DrawPolygon;
    public function void(b2Transform transform, b2Vec2* vertices, int32 vertexCount, float radius, b2HexColor color, void* context) DrawSolidPolygon;
    public function void(b2Vec2 center, float radius, b2HexColor color, void* context) DrawCircle;
    public function void(b2Transform transform, float radius, b2HexColor color, void* context) DrawSolidCircle;
    public function void(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context) DrawCapsule;
    public function void(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context) DrawSolidCapsule;
    public function void(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context) DrawSegment;
    public function void(b2Transform transform, void* context) DrawTransform;
    public function void(b2Vec2 p, float size, b2HexColor color, void* context) DrawPoint;
    public function void(b2Vec2 p, char8* s, void* context) DrawString;
    public b2AABB drawingBounds;
    public bool useDrawingBounds;
    public bool drawShapes;
    public bool drawJoints;
    public bool drawJointExtras;
    public bool drawAABBs;
    public bool drawMass;
    public bool drawContacts;
    public bool drawGraphColors;
    public bool drawContactNormals;
    public bool drawContactImpulses;
    public bool drawFrictionImpulses;
    public void* context;
}

[CLink]
public static extern b2WorldId b2CreateWorld(b2WorldDef* def);

[CLink]
public static extern void b2DestroyWorld(b2WorldId worldId);

[CLink]
public static extern bool b2World_IsValid(b2WorldId id);

[CLink]
public static extern void b2World_Step(b2WorldId worldId, float timeStep, int32 subStepCount);

[CLink]
public static extern void b2World_Draw(b2WorldId worldId, b2DebugDraw* draw);

[CLink]
public static extern b2BodyEvents b2World_GetBodyEvents(b2WorldId worldId);

[CLink]
public static extern b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId);

[CLink]
public static extern b2ContactEvents b2World_GetContactEvents(b2WorldId worldId);

[CLink]
public static extern void b2World_OverlapAABB(b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn fcn, void* context);

[CLink]
public static extern void b2World_OverlapCircle(b2WorldId worldId, b2Circle* circle, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn fcn, void* context);

[CLink]
public static extern void b2World_OverlapCapsule(b2WorldId worldId, b2Capsule* capsule, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn fcn, void* context);

[CLink]
public static extern void b2World_OverlapPolygon(b2WorldId worldId, b2Polygon* polygon, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn fcn, void* context);

[CLink]
public static extern void b2World_CastRay(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn fcn, void* context);

[CLink]
public static extern b2RayResult b2World_CastRayClosest(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter);

[CLink]
public static extern void b2World_CastCircle(b2WorldId worldId, b2Circle* circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn fcn, void* context);

[CLink]
public static extern void b2World_CastCapsule(b2WorldId worldId, b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn fcn, void* context);

[CLink]
public static extern void b2World_CastPolygon(b2WorldId worldId, b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn fcn, void* context);

[CLink]
public static extern void b2World_EnableSleeping(b2WorldId worldId, bool flag);

[CLink]
public static extern void b2World_EnableContinuous(b2WorldId worldId, bool flag);

[CLink]
public static extern void b2World_SetRestitutionThreshold(b2WorldId worldId, float value);

[CLink]
public static extern void b2World_SetHitEventThreshold(b2WorldId worldId, float value);

[CLink]
public static extern void b2World_SetCustomFilterCallback(b2WorldId worldId, b2CustomFilterFcn fcn, void* context);

[CLink]
public static extern void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn fcn, void* context);

[CLink]
public static extern void b2World_SetGravity(b2WorldId worldId, b2Vec2 gravity);

[CLink]
public static extern b2Vec2 b2World_GetGravity(b2WorldId worldId);

[CLink]
public static extern void b2World_Explode(b2WorldId worldId, b2Vec2 position, float radius, float impulse);

[CLink]
public static extern void b2World_SetContactTuning(b2WorldId worldId, float hertz, float dampingRatio, float pushVelocity);

[CLink]
public static extern void b2World_EnableWarmStarting(b2WorldId worldId, bool flag);

[CLink]
public static extern b2Profile b2World_GetProfile(b2WorldId worldId);

[CLink]
public static extern b2Counters b2World_GetCounters(b2WorldId worldId);

[CLink]
public static extern void b2World_DumpMemoryStats(b2WorldId worldId);

[CLink]
public static extern b2BodyId b2CreateBody(b2WorldId worldId, b2BodyDef* def);

[CLink]
public static extern void b2DestroyBody(b2BodyId bodyId);

[CLink]
public static extern bool b2Body_IsValid(b2BodyId id);

[CLink]
public static extern b2BodyType b2Body_GetType(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetType(b2BodyId bodyId, b2BodyType type);

[CLink]
public static extern void b2Body_SetUserData(b2BodyId bodyId, void* userData);

[CLink]
public static extern void* b2Body_GetUserData(b2BodyId bodyId);

[CLink]
public static extern b2Vec2 b2Body_GetPosition(b2BodyId bodyId);

[CLink]
public static extern b2Rot b2Body_GetRotation(b2BodyId bodyId);

[CLink]
public static extern b2Transform b2Body_GetTransform(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, b2Rot rotation);

[CLink]
public static extern b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint);

[CLink]
public static extern b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint);

[CLink]
public static extern b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector);

[CLink]
public static extern b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector);

[CLink]
public static extern b2Vec2 b2Body_GetLinearVelocity(b2BodyId bodyId);

[CLink]
public static extern float b2Body_GetAngularVelocity(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity);

[CLink]
public static extern void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity);

[CLink]
public static extern void b2Body_ApplyForce(b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake);

[CLink]
public static extern void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake);

[CLink]
public static extern void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake);

[CLink]
public static extern void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake);

[CLink]
public static extern void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake);

[CLink]
public static extern void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake);

[CLink]
public static extern float b2Body_GetMass(b2BodyId bodyId);

[CLink]
public static extern float b2Body_GetInertiaTensor(b2BodyId bodyId);

[CLink]
public static extern b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId);

[CLink]
public static extern b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData);

[CLink]
public static extern b2MassData b2Body_GetMassData(b2BodyId bodyId);

[CLink]
public static extern void b2Body_ApplyMassFromShapes(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetAutomaticMass(b2BodyId bodyId, bool automaticMass);

[CLink]
public static extern bool b2Body_GetAutomaticMass(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetLinearDamping(b2BodyId bodyId, float linearDamping);

[CLink]
public static extern float b2Body_GetLinearDamping(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping);

[CLink]
public static extern float b2Body_GetAngularDamping(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetGravityScale(b2BodyId bodyId, float gravityScale);

[CLink]
public static extern float b2Body_GetGravityScale(b2BodyId bodyId);

[CLink]
public static extern bool b2Body_IsAwake(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetAwake(b2BodyId bodyId, bool awake);

[CLink]
public static extern void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep);

[CLink]
public static extern bool b2Body_IsSleepEnabled(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetSleepThreshold(b2BodyId bodyId, float sleepVelocity);

[CLink]
public static extern float b2Body_GetSleepThreshold(b2BodyId bodyId);

[CLink]
public static extern bool b2Body_IsEnabled(b2BodyId bodyId);

[CLink]
public static extern void b2Body_Disable(b2BodyId bodyId);

[CLink]
public static extern void b2Body_Enable(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetFixedRotation(b2BodyId bodyId, bool flag);

[CLink]
public static extern bool b2Body_IsFixedRotation(b2BodyId bodyId);

[CLink]
public static extern void b2Body_SetBullet(b2BodyId bodyId, bool flag);

[CLink]
public static extern bool b2Body_IsBullet(b2BodyId bodyId);

[CLink]
public static extern void b2Body_EnableHitEvents(b2BodyId bodyId, bool enableHitEvents);

[CLink]
public static extern int32 b2Body_GetShapeCount(b2BodyId bodyId);

[CLink]
public static extern int32 b2Body_GetShapes(b2BodyId bodyId, b2ShapeId* shapeArray, int32 capacity);

[CLink]
public static extern int32 b2Body_GetJointCount(b2BodyId bodyId);

[CLink]
public static extern int32 b2Body_GetJoints(b2BodyId bodyId, b2JointId* jointArray, int32 capacity);

[CLink]
public static extern int32 b2Body_GetContactCapacity(b2BodyId bodyId);

[CLink]
public static extern int32 b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int32 capacity);

[CLink]
public static extern b2AABB b2Body_ComputeAABB(b2BodyId bodyId);

[CLink]
public static extern b2ShapeId b2CreateCircleShape(b2BodyId bodyId, b2ShapeDef* def, b2Circle* circle);

[CLink]
public static extern b2ShapeId b2CreateSegmentShape(b2BodyId bodyId, b2ShapeDef* def, b2Segment* segment);

[CLink]
public static extern b2ShapeId b2CreateCapsuleShape(b2BodyId bodyId, b2ShapeDef* def, b2Capsule* capsule);

[CLink]
public static extern b2ShapeId b2CreatePolygonShape(b2BodyId bodyId, b2ShapeDef* def, b2Polygon* polygon);

[CLink]
public static extern void b2DestroyShape(b2ShapeId shapeId);

[CLink]
public static extern bool b2Shape_IsValid(b2ShapeId id);

[CLink]
public static extern b2ShapeType b2Shape_GetType(b2ShapeId shapeId);

[CLink]
public static extern b2BodyId b2Shape_GetBody(b2ShapeId shapeId);

[CLink]
public static extern bool b2Shape_IsSensor(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_SetUserData(b2ShapeId shapeId, void* userData);

[CLink]
public static extern void* b2Shape_GetUserData(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_SetDensity(b2ShapeId shapeId, float density);

[CLink]
public static extern float b2Shape_GetDensity(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_SetFriction(b2ShapeId shapeId, float friction);

[CLink]
public static extern float b2Shape_GetFriction(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_SetRestitution(b2ShapeId shapeId, float restitution);

[CLink]
public static extern float b2Shape_GetRestitution(b2ShapeId shapeId);

[CLink]
public static extern b2Filter b2Shape_GetFilter(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_SetFilter(b2ShapeId shapeId, b2Filter filter);

[CLink]
public static extern void b2Shape_EnableSensorEvents(b2ShapeId shapeId, bool flag);

[CLink]
public static extern bool b2Shape_AreSensorEventsEnabled(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_EnableContactEvents(b2ShapeId shapeId, bool flag);

[CLink]
public static extern bool b2Shape_AreContactEventsEnabled(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_EnablePreSolveEvents(b2ShapeId shapeId, bool flag);

[CLink]
public static extern bool b2Shape_ArePreSolveEventsEnabled(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_EnableHitEvents(b2ShapeId shapeId, bool flag);

[CLink]
public static extern bool b2Shape_AreHitEventsEnabled(b2ShapeId shapeId);

[CLink]
public static extern bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point);

[CLink]
public static extern b2CastOutput b2Shape_RayCast(b2ShapeId shapeId, b2Vec2 origin, b2Vec2 translation);

[CLink]
public static extern b2Circle b2Shape_GetCircle(b2ShapeId shapeId);

[CLink]
public static extern b2Segment b2Shape_GetSegment(b2ShapeId shapeId);

[CLink]
public static extern b2SmoothSegment b2Shape_GetSmoothSegment(b2ShapeId shapeId);

[CLink]
public static extern b2Capsule b2Shape_GetCapsule(b2ShapeId shapeId);

[CLink]
public static extern b2Polygon b2Shape_GetPolygon(b2ShapeId shapeId);

[CLink]
public static extern void b2Shape_SetCircle(b2ShapeId shapeId, b2Circle* circle);

[CLink]
public static extern void b2Shape_SetCapsule(b2ShapeId shapeId, b2Capsule* capsule);

[CLink]
public static extern void b2Shape_SetSegment(b2ShapeId shapeId, b2Segment* segment);

[CLink]
public static extern void b2Shape_SetPolygon(b2ShapeId shapeId, b2Polygon* polygon);

[CLink]
public static extern b2ChainId b2Shape_GetParentChain(b2ShapeId shapeId);

[CLink]
public static extern int32 b2Shape_GetContactCapacity(b2ShapeId shapeId);

[CLink]
public static extern int32 b2Shape_GetContactData(b2ShapeId shapeId, b2ContactData* contactData, int32 capacity);

[CLink]
public static extern b2AABB b2Shape_GetAABB(b2ShapeId shapeId);

[CLink]
public static extern b2Vec2 b2Shape_GetClosestPoint(b2ShapeId shapeId, b2Vec2 target);

[CLink]
public static extern b2ChainId b2CreateChain(b2BodyId bodyId, b2ChainDef* def);

[CLink]
public static extern void b2DestroyChain(b2ChainId chainId);

[CLink]
public static extern void b2Chain_SetFriction(b2ChainId chainId, float friction);

[CLink]
public static extern void b2Chain_SetRestitution(b2ChainId chainId, float restitution);

[CLink]
public static extern bool b2Chain_IsValid(b2ChainId id);

[CLink]
public static extern void b2DestroyJoint(b2JointId jointId);

[CLink]
public static extern bool b2Joint_IsValid(b2JointId id);

[CLink]
public static extern b2JointType b2Joint_GetType(b2JointId jointId);

[CLink]
public static extern b2BodyId b2Joint_GetBodyA(b2JointId jointId);

[CLink]
public static extern b2BodyId b2Joint_GetBodyB(b2JointId jointId);

[CLink]
public static extern b2Vec2 b2Joint_GetLocalAnchorA(b2JointId jointId);

[CLink]
public static extern b2Vec2 b2Joint_GetLocalAnchorB(b2JointId jointId);

[CLink]
public static extern void b2Joint_SetCollideConnected(b2JointId jointId, bool shouldCollide);

[CLink]
public static extern bool b2Joint_GetCollideConnected(b2JointId jointId);

[CLink]
public static extern void b2Joint_SetUserData(b2JointId jointId, void* userData);

[CLink]
public static extern void* b2Joint_GetUserData(b2JointId jointId);

[CLink]
public static extern void b2Joint_WakeBodies(b2JointId jointId);

[CLink]
public static extern b2Vec2 b2Joint_GetConstraintForce(b2JointId jointId);

[CLink]
public static extern float b2Joint_GetConstraintTorque(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreateDistanceJoint(b2WorldId worldId, b2DistanceJointDef* def);

[CLink]
public static extern void b2DistanceJoint_SetLength(b2JointId jointId, float length);

[CLink]
public static extern float b2DistanceJoint_GetLength(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_EnableSpring(b2JointId jointId, bool enableSpring);

[CLink]
public static extern bool b2DistanceJoint_IsSpringEnabled(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_SetSpringHertz(b2JointId jointId, float hertz);

[CLink]
public static extern void b2DistanceJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2DistanceJoint_GetHertz(b2JointId jointId);

[CLink]
public static extern float b2DistanceJoint_GetDampingRatio(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_EnableLimit(b2JointId jointId, bool enableLimit);

[CLink]
public static extern bool b2DistanceJoint_IsLimitEnabled(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_SetLengthRange(b2JointId jointId, float minLength, float maxLength);

[CLink]
public static extern float b2DistanceJoint_GetMinLength(b2JointId jointId);

[CLink]
public static extern float b2DistanceJoint_GetMaxLength(b2JointId jointId);

[CLink]
public static extern float b2DistanceJoint_GetCurrentLength(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_EnableMotor(b2JointId jointId, bool enableMotor);

[CLink]
public static extern bool b2DistanceJoint_IsMotorEnabled(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

[CLink]
public static extern float b2DistanceJoint_GetMotorSpeed(b2JointId jointId);

[CLink]
public static extern void b2DistanceJoint_SetMaxMotorForce(b2JointId jointId, float force);

[CLink]
public static extern float b2DistanceJoint_GetMaxMotorForce(b2JointId jointId);

[CLink]
public static extern float b2DistanceJoint_GetMotorForce(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreateMotorJoint(b2WorldId worldId, b2MotorJointDef* def);

[CLink]
public static extern void b2MotorJoint_SetLinearOffset(b2JointId jointId, b2Vec2 linearOffset);

[CLink]
public static extern b2Vec2 b2MotorJoint_GetLinearOffset(b2JointId jointId);

[CLink]
public static extern void b2MotorJoint_SetAngularOffset(b2JointId jointId, float angularOffset);

[CLink]
public static extern float b2MotorJoint_GetAngularOffset(b2JointId jointId);

[CLink]
public static extern void b2MotorJoint_SetMaxForce(b2JointId jointId, float maxForce);

[CLink]
public static extern float b2MotorJoint_GetMaxForce(b2JointId jointId);

[CLink]
public static extern void b2MotorJoint_SetMaxTorque(b2JointId jointId, float maxTorque);

[CLink]
public static extern float b2MotorJoint_GetMaxTorque(b2JointId jointId);

[CLink]
public static extern void b2MotorJoint_SetCorrectionFactor(b2JointId jointId, float correctionFactor);

[CLink]
public static extern float b2MotorJoint_GetCorrectionFactor(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreateMouseJoint(b2WorldId worldId, b2MouseJointDef* def);

[CLink]
public static extern void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target);

[CLink]
public static extern b2Vec2 b2MouseJoint_GetTarget(b2JointId jointId);

[CLink]
public static extern void b2MouseJoint_SetSpringHertz(b2JointId jointId, float hertz);

[CLink]
public static extern float b2MouseJoint_GetSpringHertz(b2JointId jointId);

[CLink]
public static extern void b2MouseJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2MouseJoint_GetSpringDampingRatio(b2JointId jointId);

[CLink]
public static extern void b2MouseJoint_SetMaxForce(b2JointId jointId, float maxForce);

[CLink]
public static extern float b2MouseJoint_GetMaxForce(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreatePrismaticJoint(b2WorldId worldId, b2PrismaticJointDef* def);

[CLink]
public static extern void b2PrismaticJoint_EnableSpring(b2JointId jointId, bool enableSpring);

[CLink]
public static extern bool b2PrismaticJoint_IsSpringEnabled(b2JointId jointId);

[CLink]
public static extern void b2PrismaticJoint_SetSpringHertz(b2JointId jointId, float hertz);

[CLink]
public static extern float b2PrismaticJoint_GetSpringHertz(b2JointId jointId);

[CLink]
public static extern void b2PrismaticJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2PrismaticJoint_GetSpringDampingRatio(b2JointId jointId);

[CLink]
public static extern void b2PrismaticJoint_EnableLimit(b2JointId jointId, bool enableLimit);

[CLink]
public static extern bool b2PrismaticJoint_IsLimitEnabled(b2JointId jointId);

[CLink]
public static extern float b2PrismaticJoint_GetLowerLimit(b2JointId jointId);

[CLink]
public static extern float b2PrismaticJoint_GetUpperLimit(b2JointId jointId);

[CLink]
public static extern void b2PrismaticJoint_SetLimits(b2JointId jointId, float lower, float upper);

[CLink]
public static extern void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor);

[CLink]
public static extern bool b2PrismaticJoint_IsMotorEnabled(b2JointId jointId);

[CLink]
public static extern void b2PrismaticJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

[CLink]
public static extern float b2PrismaticJoint_GetMotorSpeed(b2JointId jointId);

[CLink]
public static extern void b2PrismaticJoint_SetMaxMotorForce(b2JointId jointId, float force);

[CLink]
public static extern float b2PrismaticJoint_GetMaxMotorForce(b2JointId jointId);

[CLink]
public static extern float b2PrismaticJoint_GetMotorForce(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreateRevoluteJoint(b2WorldId worldId, b2RevoluteJointDef* def);

[CLink]
public static extern void b2RevoluteJoint_EnableSpring(b2JointId jointId, bool enableSpring);

[CLink]
public static extern void b2RevoluteJoint_SetSpringHertz(b2JointId jointId, float hertz);

[CLink]
public static extern float b2RevoluteJoint_GetSpringHertz(b2JointId jointId);

[CLink]
public static extern void b2RevoluteJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2RevoluteJoint_GetSpringDampingRatio(b2JointId jointId);

[CLink]
public static extern float b2RevoluteJoint_GetAngle(b2JointId jointId);

[CLink]
public static extern void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit);

[CLink]
public static extern bool b2RevoluteJoint_IsLimitEnabled(b2JointId jointId);

[CLink]
public static extern float b2RevoluteJoint_GetLowerLimit(b2JointId jointId);

[CLink]
public static extern float b2RevoluteJoint_GetUpperLimit(b2JointId jointId);

[CLink]
public static extern void b2RevoluteJoint_SetLimits(b2JointId jointId, float lower, float upper);

[CLink]
public static extern void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor);

[CLink]
public static extern bool b2RevoluteJoint_IsMotorEnabled(b2JointId jointId);

[CLink]
public static extern void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

[CLink]
public static extern float b2RevoluteJoint_GetMotorSpeed(b2JointId jointId);

[CLink]
public static extern float b2RevoluteJoint_GetMotorTorque(b2JointId jointId);

[CLink]
public static extern void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque);

[CLink]
public static extern float b2RevoluteJoint_GetMaxMotorTorque(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreateWeldJoint(b2WorldId worldId, b2WeldJointDef* def);

[CLink]
public static extern void b2WeldJoint_SetLinearHertz(b2JointId jointId, float hertz);

[CLink]
public static extern float b2WeldJoint_GetLinearHertz(b2JointId jointId);

[CLink]
public static extern void b2WeldJoint_SetLinearDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2WeldJoint_GetLinearDampingRatio(b2JointId jointId);

[CLink]
public static extern void b2WeldJoint_SetAngularHertz(b2JointId jointId, float hertz);

[CLink]
public static extern float b2WeldJoint_GetAngularHertz(b2JointId jointId);

[CLink]
public static extern void b2WeldJoint_SetAngularDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2WeldJoint_GetAngularDampingRatio(b2JointId jointId);

[CLink]
public static extern b2JointId b2CreateWheelJoint(b2WorldId worldId, b2WheelJointDef* def);

[CLink]
public static extern void b2WheelJoint_EnableSpring(b2JointId jointId, bool enableSpring);

[CLink]
public static extern bool b2WheelJoint_IsSpringEnabled(b2JointId jointId);

[CLink]
public static extern void b2WheelJoint_SetSpringHertz(b2JointId jointId, float hertz);

[CLink]
public static extern float b2WheelJoint_GetSpringHertz(b2JointId jointId);

[CLink]
public static extern void b2WheelJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

[CLink]
public static extern float b2WheelJoint_GetSpringDampingRatio(b2JointId jointId);

[CLink]
public static extern void b2WheelJoint_EnableLimit(b2JointId jointId, bool enableLimit);

[CLink]
public static extern bool b2WheelJoint_IsLimitEnabled(b2JointId jointId);

[CLink]
public static extern float b2WheelJoint_GetLowerLimit(b2JointId jointId);

[CLink]
public static extern float b2WheelJoint_GetUpperLimit(b2JointId jointId);

[CLink]
public static extern void b2WheelJoint_SetLimits(b2JointId jointId, float lower, float upper);

[CLink]
public static extern void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor);

[CLink]
public static extern bool b2WheelJoint_IsMotorEnabled(b2JointId jointId);

[CLink]
public static extern void b2WheelJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

[CLink]
public static extern float b2WheelJoint_GetMotorSpeed(b2JointId jointId);

[CLink]
public static extern void b2WheelJoint_SetMaxMotorTorque(b2JointId jointId, float torque);

[CLink]
public static extern float b2WheelJoint_GetMaxMotorTorque(b2JointId jointId);

[CLink]
public static extern float b2WheelJoint_GetMotorTorque(b2JointId jointId);

}