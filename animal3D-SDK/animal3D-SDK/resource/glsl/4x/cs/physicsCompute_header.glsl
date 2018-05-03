/*
	Brian Baron		0974390
	Tyler Chermely	id
	Justin Mulkin	id

	EGP-425 Final Project

	Certificate of Authenticity
	"We certify that this work is entirely our own.  
	The assessor of this project may reproduce this project
	and provide copies to other academic staff, 
	and/or communicate a copy of this project to a plagiarism-checking service, 
	which may retain a copy of the project on its database."
*/

#version 430

// Constants
const uint MAX_RIGIDBODIES = 128;

// Enumerated data types
enum a3_ConvexHullType
{
	a3hullType_none,
	a3hullType_point,
	a3hullType_plane,
	a3hullType_box,
	a3hullType_sphere,
	a3hullType_cylinder,
	a3hullType_mesh,
};

enum a3_ConvexHullFlag
{
	a3hullFlag_none,			// 0x00
	a3hullFlag_is3D,			// 0x01
	a3hullFlag_isAxisAligned,	// 0x02
};

// generic convex hull properties
enum a3_ConvexHullProperty
{
	// boxy shape properties
	a3hullProperty_width = 0,
	a3hullProperty_height,
	a3hullProperty_depth,
	a3hullProperty_halfwidth,
	a3hullProperty_halfheight,
	a3hullProperty_halfdepth,
	a3hullProperty_halfwidthSq,
	a3hullProperty_halfheightSq,
	a3hullProperty_halfdepthSq,

	// round shape properties
	a3hullProperty_radius = 0,
	a3hullProperty_length,
	a3hullProperty_radiusSq,

	// user-defined properties
	a3hullProperty_user = a3hullProperty_maxCount_preset,
};

enum a3_Axis
{
	a3axis_x,	// index 0 in a matrix
	a3axis_y,	// index 1 in a matrix
	a3axis_z,	// index 2 in a matrix
};

// Physics structures
struct Rigidbody
{
	vec3
		position, 
		velocity, 
		acceleration, 
		momentum, 
		force;

	float
		mass,
		massInv;

	vec4
		rotation,
		torque,
		velocity_a,
		acceleration_a;

	mat3
		inertiaTensor,
		intertiaTensorInv,
		inertiaTensor_t,
		intertiaTensorInv_t;

	vec3
		centerMass,
		centerMass_t;
};

struct ConvexHull
{
	Rigidbody *rb;
	const mat4
		*transform,
		*transformInv;

	a3_ConvexHullType type;
	a3_ConvexHullFlag flag;

	a3_Axis axis;
	a3real prop[a3hullProperty_maxCount_preset + a3hullProperty_maxCount_user];
};

struct ConvexHullCollision
{
	const ConvexHull *hull_a, *hull_b;

	// list of contact points + normals
	//vec3 contact_a[a3hullContact_maxCount], contact_b[a3hullContact_maxCount];
	//vec3 normal_a[a3hullContact_maxCount], normal_b[a3hullContact_maxCount];
	unsigned int contactCount_a, contactCount_b;
};