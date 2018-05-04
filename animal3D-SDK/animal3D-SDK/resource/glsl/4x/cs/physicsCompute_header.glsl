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
layout(local_size_x = 11) in;

layout(std430, binding = 1) buffer count
{
	uint numRigidbodies;
};

layout(std430, binding = 2) buffer velocitiesTemp
{
	vec4 velocities[];
};

layout(std430, binding = 3) buffer positionsTemp
{
	vec4 positions[];
};

layout(std430, binding = 4) buffer massInvTemp
{
	float massInv[];
};

layout(std430, binding = 5) buffer typesTemp
{
	uint types[];
};

layout(std430, binding = 6) buffer charOneTemp
{
	uint charOnes[];
};

layout(std430, binding = 7) buffer charTwoTemp
{
	uint charTwos[];
};

layout(std430, binding = 8) buffer charThreeTemp
{
	uint charThrees[];
};

layout(std430, binding = 9) buffer charFourTemp
{
	uint charFours[];
};

int collisionTestSpheres(inout vec4 colPointA, inout vec4 colPointB, inout vec4 colNormalA, inout vec4 colNormalB,
	vec4 sphereCenter_a, float sphereRadius_a, vec4 sphereCenter_b, float sphereRadius_b)
{
	float sumRadii = sphereRadius_a + sphereRadius_b;
	vec4 diff_tmp = sphereCenter_a - sphereCenter_b;

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

int collisionTestSphereAABB(inout vec4 colPointA, inout vec4 colPointB, inout vec4 colNormalA, inout vec4 colNormalB,
	vec4 sphereCenter_localToAABB, float sphereRadius, vec4 aabbMinExtents, vec4 aabbMaxExtents)
{
	vec4 tmp;
	// closest point to the sphere center on the box
	tmp.x = (sphereCenter_localToAABB[0] < aabbMinExtents[0]) ? aabbMinExtents[0] : (sphereCenter_localToAABB[0] > aabbMaxExtents[0]) ? aabbMaxExtents[0] : sphereCenter_localToAABB[0];
	tmp.y = (sphereCenter_localToAABB[1] < aabbMinExtents[1]) ? aabbMinExtents[1] : (sphereCenter_localToAABB[1] > aabbMaxExtents[1]) ? aabbMaxExtents[1] : sphereCenter_localToAABB[1];
	tmp.z = (sphereCenter_localToAABB[2] < aabbMinExtents[2]) ? aabbMinExtents[2] : (sphereCenter_localToAABB[2] > aabbMaxExtents[2]) ? aabbMaxExtents[2] : sphereCenter_localToAABB[2];

	vec4 diff_tmp = tmp - sphereCenter_localToAABB;
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
	uint coord = gl_GlobalInvocationID.x;

	for(int i = 0; i < numRigidbodies; i++)
	{
		vec4 normalA, normalB, colPointA, colPointB;
		int collided = 0;
		if(coord != i)
		{
			if(types[coord] == 0)
			{
				if(types[i] == 0)
				{
					collided = collisionTestSpheres(colPointA, colPointB, normalA, normalB, positions[coord], charOnes[coord],
						positions[i], charOnes[i]);
				}
				else if(types[i] == 1)
				{
					vec4 minB, maxB;

					minB.x = positions[i].x - charOnes[i];
					maxB.x = positions[i].x + charOnes[i];

					minB.y = positions[i].y - charTwos[i];
					maxB.y = positions[i].y + charTwos[i];

					minB.z = positions[i].z - charThrees[i];
					maxB.z = positions[i].z + charThrees[i];

					collided = collisionTestSphereAABB(colPointA, colPointB, normalA, normalB, positions[coord], charOnes[coord],
						minB, maxB);
				}
			}
			else if(types[coord] == 1)
			{
				if(types[i] == 0)
				{
					vec4 minB, maxB;

					minB.x = positions[coord].x - charOnes[coord];
					maxB.x = positions[coord].x + charOnes[coord];

					minB.y = positions[coord].y - charTwos[coord];
					maxB.y = positions[coord].y + charTwos[coord];

					minB.z = positions[coord].z - charThrees[coord];
					maxB.z = positions[coord].z + charThrees[coord];

					collided = collisionTestSphereAABB(colPointA, colPointB, normalA, normalB, positions[i], charOnes[i],
						minB, maxB);
				}
			}
		}

		if(collided != 0)
		{
			// http://www.chrishecker.com/images/e/e7/Gdmphys3.pdf
			//vec4 rVel;
			//rVel =  velocities[coord] - velocities[i];

			//float j1 = (-2 * dot(rVel, normalA)) / (dot(normalA, 
			//	normalA)*(massInv[coord] + massInv[i]));

			velocities[coord] *= -1;//normalA * (j1 * massInv[coord]);
			velocities[i] *= -1;//normalB * (j1 * massInv[i]);
		}
	}
}