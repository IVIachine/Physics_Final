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
/*layout(local_size_x = 1, local_size_y = 1) in;

struct Rigidbody
{
	vec3 velocity;
	vec3 position;
	uint characteristicOne, characteristicTwo, characteristicThree, characteristicFour;
};

layout(std430, binding = 0) buffer rigidBodies
{
    Rigidbody rigidBodyData[];
};

int collisionTestSpheres(inout vec3 colPointA, inout vec3 colPointB, inout vec3 colNormalA, inout vec3 colNormalB,
	vec3 sphereCenter_a, float sphereRadius_a, vec3 sphereCenter_b, float sphereRadius_b)
{
	float sumRadii = sphereRadius_a + sphereRadius_b;
	vec3 diff_tmp = sphereCenter_a - sphereCenter_b;

	if (length(diff_tmp) <= sumRadii)
	{
		// generate contacts
		normalize(diff_tmp);
		colNormalA = diff_tmp;
		
		colPointB = diff_tmp * sphereRadius_b;
		colPointB += sphereCenter_b;
		
		diff_tmp *= -1;
		colNormalB = diff_tmp;
		colPointA = diff_tmp * sphereRadius_a;
		colPointA += sphereCenter_a;

		return 1;
	}
	return 0;
}

int collisionTestSphereAABB(inout vec3 colPointA, inout vec3 colPointB, inout vec3 colNormalA, inout vec3 colNormalB,
	vec3 sphereCenter_localToAABB, float sphereRadius, vec3 aabbMinExtents, vec3 aabbMaxExtents)
{
	vec3 tmp;
	// closest point to the sphere center on the box
	tmp.x = (sphereCenter_localToAABB[0] < aabbMinExtents[0]) ? aabbMinExtents[0] : (sphereCenter_localToAABB[0] > aabbMaxExtents[0]) ? aabbMaxExtents[0] : sphereCenter_localToAABB[0];
	tmp.y = (sphereCenter_localToAABB[1] < aabbMinExtents[1]) ? aabbMinExtents[1] : (sphereCenter_localToAABB[1] > aabbMaxExtents[1]) ? aabbMaxExtents[1] : sphereCenter_localToAABB[1];
	tmp.z = (sphereCenter_localToAABB[2] < aabbMinExtents[2]) ? aabbMinExtents[2] : (sphereCenter_localToAABB[2] > aabbMaxExtents[2]) ? aabbMaxExtents[2] : sphereCenter_localToAABB[2];

	vec3 diff_tmp = tmp - sphereCenter_localToAABB;
	if (length(diff_tmp) <=  sphereRadius)
	{
		// sphere is a
		// box is b
		colPointA = colPointB = tmp;
		normalize(diff_tmp);

		colNormalA = diff_tmp;
		colNormalB = -diff_tmp;

		return 1;
	}

	return 0;
}*/

void main() {
	return 0;
}