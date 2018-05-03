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
layout(local_size_x = 1) in;

struct Rigidbody
{
	vec3 velocity;
	vec3 position;
	float massInv;
	uint type;
	uint characteristicOne, characteristicTwo, characteristicThree, characteristicFour;
};

layout(std430, binding = 2) buffer rigidBodies
{
	uint numRigidbodies;
    Rigidbody rigidBodyData[];
} rigidbodies;

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
}

void main() {
	int coord = ivec2(gl_GlobalInvocationID.xy).x;

	rigidbodies.numRigidbodies = 1000;

	for(int i = 0; i < rigidbodies.numRigidbodies; i++)
	{
		vec3 normalA, normalB, colPointA, colPointB;
		int collided = 0;
		if(coord != i)
		{
			if(rigidbodies.rigidBodyData[coord].type == 0)
			{
				if(rigidbodies.rigidBodyData[i].type == 0)
				{
					collided = collisionTestSpheres(colPointA, colPointB, normalA, normalB, rigidbodies.rigidBodyData[coord].position, rigidbodies.rigidBodyData[coord].characteristicOne,
						rigidbodies.rigidBodyData[i].position, rigidbodies.rigidBodyData[i].characteristicOne);
				}
				else if(rigidbodies.rigidBodyData[i].type == 1)
				{
					vec3 minB, maxB;

					minB.x = rigidbodies.rigidBodyData[i].position.x - rigidbodies.rigidBodyData[i].characteristicOne;
					maxB.x = rigidbodies.rigidBodyData[i].position.x +rigidbodies.rigidBodyData[i].characteristicOne;

					minB.y = rigidbodies.rigidBodyData[i].position.y - rigidbodies.rigidBodyData[i].characteristicTwo;
					maxB.y = rigidbodies.rigidBodyData[i].position.y + rigidbodies.rigidBodyData[i].characteristicTwo;

					minB.z = rigidbodies.rigidBodyData[i].position.z - rigidbodies.rigidBodyData[i].characteristicThree;
					maxB.z = rigidbodies.rigidBodyData[i].position.z + rigidbodies.rigidBodyData[i].characteristicThree;

					collided = collisionTestSphereAABB(colPointA, colPointB, normalA, normalB, rigidbodies.rigidBodyData[coord].position, rigidbodies.rigidBodyData[coord].characteristicOne,
						minB, maxB);
				}
			}
			else if(rigidbodies.rigidBodyData[coord].type == 1)
			{
				if(rigidbodies.rigidBodyData[i].type == 0)
				{
					vec3 minB, maxB;

					minB.x = rigidbodies.rigidBodyData[coord].position.x - rigidbodies.rigidBodyData[coord].characteristicOne;
					maxB.x = rigidbodies.rigidBodyData[coord].position.x +rigidbodies.rigidBodyData[coord].characteristicOne;
										   
					minB.y = rigidbodies.rigidBodyData[coord].position.y - rigidbodies.rigidBodyData[coord].characteristicTwo;
					maxB.y = rigidbodies.rigidBodyData[coord].position.y + rigidbodies.rigidBodyData[coord].characteristicTwo;
										   
					minB.z = rigidbodies.rigidBodyData[coord].position.z - rigidbodies.rigidBodyData[coord].characteristicThree;
					maxB.z = rigidbodies.rigidBodyData[coord].position.z + rigidbodies.rigidBodyData[coord].characteristicThree;

					collided = collisionTestSphereAABB(colPointA, colPointB, normalA, normalB, rigidbodies.rigidBodyData[i].position, rigidbodies.rigidBodyData[i].characteristicOne,
						minB, maxB);
				}
			}
		}

		if(collided != 0)
		{
				// http://www.chrishecker.com/images/e/e7/Gdmphys3.pdf
				vec3 rVel;
				rVel =  rigidbodies.rigidBodyData[coord].velocity - rigidbodies.rigidBodyData[i].velocity;

				float j1 = (-2 * dot(rVel, normalA)) / (dot(normalA, 
					normalA)*(rigidbodies.rigidBodyData[coord].massInv + rigidbodies.rigidBodyData[i].massInv));

				rigidbodies.rigidBodyData[coord].velocity = normalA * (j1 * rigidbodies.rigidBodyData[coord].massInv);
				rigidbodies.rigidBodyData[i].velocity =  normalB * (j1 * rigidbodies.rigidBodyData[i].massInv);
		}
	}
}