using System;
namespace Box2DBeef;

public function void* b2AllocFcn(uint32 size, int32 alignment);

public function void b2FreeFcn(void* mem);

public function int32 b2AssertFcn(char8* condition, char8* fileName, int32 lineNumber);

[CRepr]
public struct b2Version : this(int32 major, int32 minor, int32 revision);

[CRepr]
public struct b2Timer : this(int64 start);

[CRepr]
public struct b2Vec2 : this(float x, float y);

[CRepr]
public struct b2Rot : this(float c, float s);

[CRepr]
public struct b2Transform : this(b2Vec2 p, b2Rot q);

[CRepr]
public struct b2Mat22 : this(b2Vec2 cx, b2Vec2 cy);

[CRepr]
public struct b2AABB : this(b2Vec2 lowerBound, b2Vec2 upperBound);

[CRepr]
public struct b2RayCastInput : this(b2Vec2 origin, b2Vec2 translation, float maxFraction);

[CRepr]
public struct b2ShapeCastInput : this(b2Vec2[8] points, int32 count, float radius, b2Vec2 translation, float maxFraction);

[CRepr]
public struct b2CastOutput : this(b2Vec2 normal, b2Vec2 point, float fraction, int32 iterations, bool hit);

[CRepr]
public struct b2MassData : this(float mass, b2Vec2 center, float I);

[CRepr]
public struct b2Circle : this(b2Vec2 center, float radius);

[CRepr]
public struct b2Capsule : this(b2Vec2 center1, b2Vec2 center2, float radius);

[CRepr]
public struct b2Polygon : this(b2Vec2[8] vertices, b2Vec2[8] normals, b2Vec2 centroid, float radius, int32 count);

[CRepr]
public struct b2Segment : this(b2Vec2 point1, b2Vec2 point2);

[CRepr]
public struct b2SmoothSegment : this(b2Vec2 ghost1, b2Segment segment, b2Vec2 ghost2, int32 chainId);

[CRepr]
public struct b2Hull : this(b2Vec2[8] points, int32 count);

[CRepr]
public struct b2SegmentDistanceResult : this(b2Vec2 closest1, b2Vec2 closest2, float fraction1, float fraction2, float distanceSquared);

[CRepr]
public struct b2DistanceProxy : this(b2Vec2[8] points, int32 count, float radius);

[CRepr]
public struct b2DistanceCache : this(float metric, uint16 count, uint8[3] indexA, uint8[3] indexB);

[CRepr]
public struct b2DistanceInput : this(b2DistanceProxy proxyA, b2DistanceProxy proxyB, b2Transform transformA, b2Transform transformB, bool useRadii);

[CRepr]
public struct b2DistanceOutput : this(b2Vec2 pointA, b2Vec2 pointB, float distance, int32 iterations);

[CRepr]
public struct b2ShapeCastPairInput : this(b2DistanceProxy proxyA, b2DistanceProxy proxyB, b2Transform transformA, b2Transform transformB, b2Vec2 translationB, float maxFraction);

[CRepr]
public struct b2Sweep : this(b2Vec2 localCenter, b2Vec2 c1, b2Vec2 c2, b2Rot q1, b2Rot q2);

[CRepr]
public struct b2TOIInput : this(b2DistanceProxy proxyA, b2DistanceProxy proxyB, b2Sweep sweepA, b2Sweep sweepB, float tMax);

[CRepr, AllowDuplicates]
public enum b2TOIState : int32
{
    b2_toiStateUnknown = 0,
    b2_toiStateFailed = 1,
    b2_toiStateOverlapped = 2,
    b2_toiStateHit = 3,
    b2_toiStateSeparated = 4,
}

[CRepr]
public struct b2TOIOutput : this(b2TOIState state, float t);

[CRepr]
public struct b2ManifoldPoint : this(b2Vec2 point, b2Vec2 anchorA, b2Vec2 anchorB, float separation, float normalImpulse, float tangentImpulse, float maxNormalImpulse, float normalVelocity, uint16 id, bool persisted);

[CRepr]
public struct b2Manifold : this(b2ManifoldPoint[2] points, b2Vec2 normal, int32 pointCount);

[CRepr]
public struct b2TreeNode : this(b2AABB aabb, uint32 categoryBits, int32 child1, int32 child2, int32 userData, int16 height, bool enlarged, char8[9] pad);

[CRepr]
public struct b2DynamicTree : this(b2TreeNode* nodes, int32 root, int32 nodeCount, int32 nodeCapacity, int32 freeList, int32 proxyCount, int32* leafIndices, b2AABB* leafBoxes, b2Vec2* leafCenters, int32* binIndices, int32 rebuildCapacity);

public function bool b2TreeQueryCallbackFcn(int32 proxyId, int32 userData, void* context);

public function float b2TreeRayCastCallbackFcn(b2RayCastInput* input, int32 proxyId, int32 userData, void* context);

public function float b2TreeShapeCastCallbackFcn(b2ShapeCastInput* input, int32 proxyId, int32 userData, void* context);

[CRepr]
public struct b2WorldId : this(uint16 index1, uint16 revision);

[CRepr]
public struct b2BodyId : this(int32 index1, uint16 world0, uint16 revision);

[CRepr]
public struct b2ShapeId : this(int32 index1, uint16 world0, uint16 revision);

[CRepr]
public struct b2JointId : this(int32 index1, uint16 world0, uint16 revision);

[CRepr]
public struct b2ChainId : this(int32 index1, uint16 world0, uint16 revision);

public function void b2TaskCallback(int32 startIndex, int32 endIndex, uint32 workerIndex, void* taskContext);

public function void* b2EnqueueTaskCallback(b2TaskCallback task, int32 itemCount, int32 minRange, void* taskContext, void* userContext);

public function void b2FinishTaskCallback(void* userTask, void* userContext);

[CRepr]
public struct b2RayResult : this(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, bool hit);

[CRepr]
public struct b2WorldDef : this(b2Vec2 gravity, float restitutionThreshold, float contactPushoutVelocity, float hitEventThreshold, float contactHertz, float contactDampingRatio, float jointHertz, float jointDampingRatio, bool enableSleep, bool enableContinous, int32 workerCount, b2EnqueueTaskCallback enqueueTask, b2FinishTaskCallback finishTask, void* userTaskContext, int32 internalValue);

[CRepr, AllowDuplicates]
public enum b2BodyType : int32
{
    b2_staticBody = 0,
    b2_kinematicBody = 1,
    b2_dynamicBody = 2,
    b2_bodyTypeCount = 3,
}

[CRepr]
public struct b2BodyDef : this(b2BodyType type, b2Vec2 position, float angle, b2Vec2 linearVelocity, float angularVelocity, float linearDamping, float angularDamping, float gravityScale, float sleepThreshold, void* userData, bool enableSleep, bool isAwake, bool fixedRotation, bool isBullet, bool isEnabled, bool automaticMass, int32 internalValue);

[CRepr]
public struct b2Filter : this(uint32 categoryBits, uint32 maskBits, int32 groupIndex);

[CRepr]
public struct b2QueryFilter : this(uint32 categoryBits, uint32 maskBits);

[CRepr, AllowDuplicates]
public enum b2ShapeType : int32
{
    b2_circleShape = 0,
    b2_capsuleShape = 1,
    b2_segmentShape = 2,
    b2_polygonShape = 3,
    b2_smoothSegmentShape = 4,
    b2_shapeTypeCount = 5,
}

[CRepr]
public struct b2ShapeDef : this(void* userData, float friction, float restitution, float density, b2Filter filter, bool isSensor, bool enableSensorEvents, bool enableContactEvents, bool enableHitEvents, bool enablePreSolveEvents, bool forceContactCreation, int32 internalValue);

[CRepr]
public struct b2ChainDef : this(void* userData, b2Vec2* points, int32 count, float friction, float restitution, b2Filter filter, bool isLoop, int32 internalValue);

[CRepr]
public struct b2Profile : this(float step, float pairs, float collide, float solve, float buildIslands, float solveConstraints, float prepareTasks, float solverTasks, float prepareConstraints, float integrateVelocities, float warmStart, float solveVelocities, float integratePositions, float relaxVelocities, float applyRestitution, float storeImpulses, float finalizeBodies, float splitIslands, float sleepIslands, float hitEvents, float broadphase, float continuous);

[CRepr]
public struct b2Counters : this(int32 staticBodyCount, int32 bodyCount, int32 shapeCount, int32 contactCount, int32 jointCount, int32 islandCount, int32 stackUsed, int32 staticTreeHeight, int32 treeHeight, int32 byteCount, int32 taskCount, int32[12] colorCounts);

[CRepr, AllowDuplicates]
public enum b2JointType : int32
{
    b2_distanceJoint = 0,
    b2_motorJoint = 1,
    b2_mouseJoint = 2,
    b2_prismaticJoint = 3,
    b2_revoluteJoint = 4,
    b2_weldJoint = 5,
    b2_wheelJoint = 6,
}

[CRepr]
public struct b2DistanceJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 localAnchorA, b2Vec2 localAnchorB, float length, bool enableSpring, float hertz, float dampingRatio, bool enableLimit, float minLength, float maxLength, bool enableMotor, float maxMotorForce, float motorSpeed, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2MotorJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 linearOffset, float angularOffset, float maxForce, float maxTorque, float correctionFactor, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2MouseJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 target, float hertz, float dampingRatio, float maxForce, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2PrismaticJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 localAnchorA, b2Vec2 localAnchorB, b2Vec2 localAxisA, float referenceAngle, bool enableSpring, float hertz, float dampingRatio, bool enableLimit, float lowerTranslation, float upperTranslation, bool enableMotor, float maxMotorForce, float motorSpeed, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2RevoluteJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 localAnchorA, b2Vec2 localAnchorB, float referenceAngle, bool enableSpring, float hertz, float dampingRatio, bool enableLimit, float lowerAngle, float upperAngle, bool enableMotor, float maxMotorTorque, float motorSpeed, float drawSize, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2WeldJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 localAnchorA, b2Vec2 localAnchorB, float referenceAngle, float linearHertz, float angularHertz, float linearDampingRatio, float angularDampingRatio, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2WheelJointDef : this(b2BodyId bodyIdA, b2BodyId bodyIdB, b2Vec2 localAnchorA, b2Vec2 localAnchorB, b2Vec2 localAxisA, bool enableSpring, float hertz, float dampingRatio, bool enableLimit, float lowerTranslation, float upperTranslation, bool enableMotor, float maxMotorTorque, float motorSpeed, bool collideConnected, void* userData, int32 internalValue);

[CRepr]
public struct b2SensorBeginTouchEvent : this(b2ShapeId sensorShapeId, b2ShapeId visitorShapeId);

[CRepr]
public struct b2SensorEndTouchEvent : this(b2ShapeId sensorShapeId, b2ShapeId visitorShapeId);

[CRepr]
public struct b2SensorEvents : this(b2SensorBeginTouchEvent* beginEvents, b2SensorEndTouchEvent* endEvents, int32 beginCount, int32 endCount);

[CRepr]
public struct b2ContactBeginTouchEvent : this(b2ShapeId shapeIdA, b2ShapeId shapeIdB);

[CRepr]
public struct b2ContactEndTouchEvent : this(b2ShapeId shapeIdA, b2ShapeId shapeIdB);

[CRepr]
public struct b2ContactHitEvent : this(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Vec2 point, b2Vec2 normal, float approachSpeed);

[CRepr]
public struct b2ContactEvents : this(b2ContactBeginTouchEvent* beginEvents, b2ContactEndTouchEvent* endEvents, b2ContactHitEvent* hitEvents, int32 beginCount, int32 endCount, int32 hitCount);

[CRepr]
public struct b2BodyMoveEvent : this(b2Transform transform, b2BodyId bodyId, void* userData, bool fellAsleep);

[CRepr]
public struct b2BodyEvents : this(b2BodyMoveEvent* moveEvents, int32 moveCount);

[CRepr]
public struct b2ContactData : this(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold manifold);

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
}

[CRepr]
public struct b2DebugDraw : this(function void(b2Vec2* vertices, int32 vertexCount, b2HexColor color, void* context) DrawPolygon, function void(b2Transform transform, b2Vec2* vertices, int32 vertexCount, float radius, b2HexColor color, void* context) DrawSolidPolygon, function void(b2Vec2 center, float radius, b2HexColor color, void* context) DrawCircle, function void(b2Transform transform, float radius, b2HexColor color, void* context) DrawSolidCircle, function void(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context) DrawCapsule, function void(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context) DrawSolidCapsule, function void(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context) DrawSegment, function void(b2Transform transform, void* context) DrawTransform, function void(b2Vec2 p, float size, b2HexColor color, void* context) DrawPoint, function void(b2Vec2 p, char8* s, void* context) DrawString, b2AABB drawingBounds, bool useDrawingBounds, bool drawShapes, bool drawJoints, bool drawJointExtras, bool drawAABBs, bool drawMass, bool drawContacts, bool drawGraphColors, bool drawContactNormals, bool drawContactImpulses, bool drawFrictionImpulses, void* context);

static {
    [CLink]
    public static extern void b2SetAllocator(b2AllocFcn allocFcn, b2FreeFcn freeFcn);

    [CLink]
    public static extern int32 b2GetByteCount();

    [CLink]
    public static extern void b2SetAssertFcn(b2AssertFcn assertFcn);

    [CLink]
    public static extern b2Version b2GetVersion();

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

    [CLink]
    public static extern float b2MinFloat();

    [CLink]
    public static extern float b2MaxFloat();

    [CLink]
    public static extern float b2AbsFloat();

    [CLink]
    public static extern float b2ClampFloat();

    [CLink]
    public static extern int32 b2MinInt();

    [CLink]
    public static extern int32 b2MaxInt();

    [CLink]
    public static extern int32 b2AbsInt();

    [CLink]
    public static extern int32 b2ClampInt();

    [CLink]
    public static extern float b2Dot();

    [CLink]
    public static extern float b2Cross();

    [CLink]
    public static extern b2Vec2 b2CrossVS();

    [CLink]
    public static extern b2Vec2 b2CrossSV();

    [CLink]
    public static extern b2Vec2 b2LeftPerp();

    [CLink]
    public static extern b2Vec2 b2RightPerp();

    [CLink]
    public static extern b2Vec2 b2Add();

    [CLink]
    public static extern b2Vec2 b2Sub();

    [CLink]
    public static extern b2Vec2 b2Neg();

    [CLink]
    public static extern b2Vec2 b2Lerp();

    [CLink]
    public static extern b2Vec2 b2Mul();

    [CLink]
    public static extern b2Vec2 b2MulSV();

    [CLink]
    public static extern b2Vec2 b2MulAdd();

    [CLink]
    public static extern b2Vec2 b2MulSub();

    [CLink]
    public static extern b2Vec2 b2Abs();

    [CLink]
    public static extern b2Vec2 b2Min();

    [CLink]
    public static extern b2Vec2 b2Max();

    [CLink]
    public static extern b2Vec2 b2Clamp();

    [CLink]
    public static extern float b2Length();

    [CLink]
    public static extern float b2LengthSquared();

    [CLink]
    public static extern float b2Distance();

    [CLink]
    public static extern float b2DistanceSquared();

    [CLink]
    public static extern b2Rot b2MakeRot();

    [CLink]
    public static extern b2Rot b2NormalizeRot();

    [CLink]
    public static extern bool b2IsNormalized();

    [CLink]
    public static extern b2Rot b2NLerp();

    [CLink]
    public static extern b2Rot b2IntegrateRotation();

    [CLink]
    public static extern float b2ComputeAngularVelocity();

    [CLink]
    public static extern float b2Rot_GetAngle();

    [CLink]
    public static extern b2Vec2 b2Rot_GetXAxis();

    [CLink]
    public static extern b2Vec2 b2Rot_GetYAxis();

    [CLink]
    public static extern b2Rot b2MulRot();

    [CLink]
    public static extern b2Rot b2InvMulRot();

    [CLink]
    public static extern float b2RelativeAngle();

    [CLink]
    public static extern float b2UnwindAngle();

    [CLink]
    public static extern b2Vec2 b2RotateVector();

    [CLink]
    public static extern b2Vec2 b2InvRotateVector();

    [CLink]
    public static extern b2Vec2 b2TransformPoint();

    [CLink]
    public static extern b2Vec2 b2InvTransformPoint();

    [CLink]
    public static extern b2Transform b2MulTransforms();

    [CLink]
    public static extern b2Transform b2InvMulTransforms();

    [CLink]
    public static extern b2Vec2 b2MulMV();

    [CLink]
    public static extern b2Mat22 b2GetInverse22();

    [CLink]
    public static extern b2Vec2 b2Solve22();

    [CLink]
    public static extern bool b2AABB_Contains();

    [CLink]
    public static extern b2Vec2 b2AABB_Center();

    [CLink]
    public static extern b2Vec2 b2AABB_Extents();

    [CLink]
    public static extern b2AABB b2AABB_Union();

    [CLink]
    public static extern bool b2IsValid();

    [CLink]
    public static extern bool b2Vec2_IsValid();

    [CLink]
    public static extern bool b2Rot_IsValid();

    [CLink]
    public static extern bool b2AABB_IsValid();

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

    [CLink]
    public static extern bool b2IsValidRay();

    [CLink]
    public static extern b2Polygon b2MakePolygon(b2Hull* hull, float radius);

    [CLink]
    public static extern b2Polygon b2MakeOffsetPolygon(b2Hull* hull, float radius, b2Transform transform);

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
    public static extern bool b2PointInCircle();

    [CLink]
    public static extern bool b2PointInCapsule();

    [CLink]
    public static extern bool b2PointInPolygon();

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
    public static extern bool b2ValidateHull();

    [CLink]
    public static extern b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2);

    [CLink]
    public static extern b2DistanceOutput b2ShapeDistance(b2DistanceCache* cache, b2DistanceInput* input);

    [CLink]
    public static extern b2CastOutput b2ShapeCast(b2ShapeCastPairInput* input);

    [CLink]
    public static extern b2DistanceProxy b2MakeProxy(b2Vec2* vertices, int32 count, float radius);

    [CLink]
    public static extern b2Transform b2GetSweepTransform(b2Sweep* sweep, float time);

    [CLink]
    public static extern b2TOIOutput b2TimeOfImpact(b2TOIInput* input);

    [CLink]
    public static extern b2Manifold b2CollideCircles(b2Circle* circleA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

    [CLink]
    public static extern b2Manifold b2CollideCapsuleAndCircle(b2Capsule* capsuleA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

    [CLink]
    public static extern b2Manifold b2CollideSegmentAndCircle(b2Segment* segmentA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

    [CLink]
    public static extern b2Manifold b2CollidePolygonAndCircle(b2Polygon* polygonA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

    [CLink]
    public static extern b2Manifold b2CollideCapsules(b2Capsule* capsuleA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB, b2DistanceCache* cache);

    [CLink]
    public static extern b2Manifold b2CollideSegmentAndCapsule(b2Segment* segmentA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB, b2DistanceCache* cache);

    [CLink]
    public static extern b2Manifold b2CollidePolygonAndCapsule(b2Polygon* polygonA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB, b2DistanceCache* cache);

    [CLink]
    public static extern b2Manifold b2CollidePolygons(b2Polygon* polyA, b2Transform xfA, b2Polygon* polyB, b2Transform xfB, b2DistanceCache* cache);

    [CLink]
    public static extern b2Manifold b2CollideSegmentAndPolygon(b2Segment* segmentA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB, b2DistanceCache* cache);

    [CLink]
    public static extern b2Manifold b2CollideSmoothSegmentAndCircle(b2SmoothSegment* smoothSegmentA, b2Transform xfA, b2Circle* circleB, b2Transform xfB);

    [CLink]
    public static extern b2Manifold b2CollideSmoothSegmentAndCapsule(b2SmoothSegment* smoothSegmentA, b2Transform xfA, b2Capsule* capsuleB, b2Transform xfB, b2DistanceCache* cache);

    [CLink]
    public static extern b2Manifold b2CollideSmoothSegmentAndPolygon(b2SmoothSegment* smoothSegmentA, b2Transform xfA, b2Polygon* polygonB, b2Transform xfB, b2DistanceCache* cache);

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

    [CLink]
    public static extern void b2DynamicTree_Query(b2DynamicTree* tree, b2AABB aabb, uint32 maskBits, b2TreeQueryCallbackFcn callback, void* context);

    [CLink]
    public static extern void b2DynamicTree_RayCast(b2DynamicTree* tree, b2RayCastInput* input, uint32 maskBits, b2TreeRayCastCallbackFcn callback, void* context);

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
    public static extern int32 b2DynamicTree_GetUserData();

    [CLink]
    public static extern b2AABB b2DynamicTree_GetAABB();

    [CLink]
    public static extern b2WorldDef b2DefaultWorldDef();

    [CLink]
    public static extern b2BodyDef b2DefaultBodyDef();

    [CLink]
    public static extern b2Filter b2DefaultFilter();

    [CLink]
    public static extern b2QueryFilter b2DefaultQueryFilter();

    [CLink]
    public static extern b2ShapeDef b2DefaultShapeDef();

    [CLink]
    public static extern b2ChainDef b2DefaultChainDef();

    [CLink]
    public static extern b2DistanceJointDef b2DefaultDistanceJointDef();

    [CLink]
    public static extern b2MotorJointDef b2DefaultMotorJointDef();

    [CLink]
    public static extern b2MouseJointDef b2DefaultMouseJointDef();

    [CLink]
    public static extern b2PrismaticJointDef b2DefaultPrismaticJointDef();

    [CLink]
    public static extern b2RevoluteJointDef b2DefaultRevoluteJointDef();

    [CLink]
    public static extern b2WeldJointDef b2DefaultWeldJointDef();

    [CLink]
    public static extern b2WheelJointDef b2DefaultWheelJointDef();

    [CLink]
    public static extern b2WorldId b2CreateWorld(b2WorldDef* def);

    [CLink]
    public static extern void b2DestroyWorld(b2WorldId worldId);

    [CLink]
    public static extern bool b2World_IsValid();

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
    public static extern bool b2Body_IsValid();

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
    public static extern float b2Body_GetAngle(b2BodyId bodyId);

    [CLink]
    public static extern b2Transform b2Body_GetTransform(b2BodyId bodyId);

    [CLink]
    public static extern void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle);

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
    public static extern bool b2Body_GetAutomaticMass();

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
    public static extern bool b2Body_IsAwake();

    [CLink]
    public static extern void b2Body_SetAwake(b2BodyId bodyId, bool awake);

    [CLink]
    public static extern void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep);

    [CLink]
    public static extern bool b2Body_IsSleepEnabled();

    [CLink]
    public static extern void b2Body_SetSleepThreshold(b2BodyId bodyId, float sleepVelocity);

    [CLink]
    public static extern float b2Body_GetSleepThreshold(b2BodyId bodyId);

    [CLink]
    public static extern bool b2Body_IsEnabled();

    [CLink]
    public static extern void b2Body_Disable(b2BodyId bodyId);

    [CLink]
    public static extern void b2Body_Enable(b2BodyId bodyId);

    [CLink]
    public static extern void b2Body_SetFixedRotation(b2BodyId bodyId, bool flag);

    [CLink]
    public static extern bool b2Body_IsFixedRotation();

    [CLink]
    public static extern void b2Body_SetBullet(b2BodyId bodyId, bool flag);

    [CLink]
    public static extern bool b2Body_IsBullet();

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
    public static extern bool b2Shape_IsValid();

    [CLink]
    public static extern b2ShapeType b2Shape_GetType(b2ShapeId shapeId);

    [CLink]
    public static extern b2BodyId b2Shape_GetBody(b2ShapeId shapeId);

    [CLink]
    public static extern bool b2Shape_IsSensor();

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
    public static extern bool b2Shape_AreSensorEventsEnabled();

    [CLink]
    public static extern void b2Shape_EnableContactEvents(b2ShapeId shapeId, bool flag);

    [CLink]
    public static extern bool b2Shape_AreContactEventsEnabled();

    [CLink]
    public static extern void b2Shape_EnablePreSolveEvents(b2ShapeId shapeId, bool flag);

    [CLink]
    public static extern bool b2Shape_ArePreSolveEventsEnabled();

    [CLink]
    public static extern void b2Shape_EnableHitEvents(b2ShapeId shapeId, bool flag);

    [CLink]
    public static extern bool b2Shape_AreHitEventsEnabled();

    [CLink]
    public static extern bool b2Shape_TestPoint();

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
    public static extern bool b2Chain_IsValid();

    [CLink]
    public static extern void b2DestroyJoint(b2JointId jointId);

    [CLink]
    public static extern bool b2Joint_IsValid();

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
    public static extern bool b2Joint_GetCollideConnected();

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
    public static extern bool b2DistanceJoint_IsSpringEnabled();

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
    public static extern bool b2DistanceJoint_IsLimitEnabled();

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
    public static extern bool b2DistanceJoint_IsMotorEnabled();

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
    public static extern bool b2PrismaticJoint_IsSpringEnabled();

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
    public static extern bool b2PrismaticJoint_IsLimitEnabled();

    [CLink]
    public static extern float b2PrismaticJoint_GetLowerLimit(b2JointId jointId);

    [CLink]
    public static extern float b2PrismaticJoint_GetUpperLimit(b2JointId jointId);

    [CLink]
    public static extern void b2PrismaticJoint_SetLimits(b2JointId jointId, float lower, float upper);

    [CLink]
    public static extern void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor);

    [CLink]
    public static extern bool b2PrismaticJoint_IsMotorEnabled();

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
    public static extern bool b2RevoluteJoint_IsLimitEnabled();

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
    public static extern float b2RevoluteJoint_GetLowerLimit(b2JointId jointId);

    [CLink]
    public static extern float b2RevoluteJoint_GetUpperLimit(b2JointId jointId);

    [CLink]
    public static extern void b2RevoluteJoint_SetLimits(b2JointId jointId, float lower, float upper);

    [CLink]
    public static extern void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor);

    [CLink]
    public static extern bool b2RevoluteJoint_IsMotorEnabled();

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
    public static extern bool b2WheelJoint_IsSpringEnabled();

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
    public static extern bool b2WheelJoint_IsLimitEnabled();

    [CLink]
    public static extern float b2WheelJoint_GetLowerLimit(b2JointId jointId);

    [CLink]
    public static extern float b2WheelJoint_GetUpperLimit(b2JointId jointId);

    [CLink]
    public static extern void b2WheelJoint_SetLimits(b2JointId jointId, float lower, float upper);

    [CLink]
    public static extern void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor);

    [CLink]
    public static extern bool b2WheelJoint_IsMotorEnabled();

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