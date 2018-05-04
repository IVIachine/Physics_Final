/*This file
was
modified by

Brian Baron
Tyler Chermely
Justin Mulkin

with permission
from author.*/

/*
	Copyright 2011-2018 Daniel S. Buckstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
	animal3D SDK: Minimal 3D Animation Framework
	By Daniel S. Buckstein

	a3_PhysicsWorld.c/.cpp
	Physics world function implementations.
*/

/*
* IDs: 0955181 and 0967813
* EGP 425-01 Project 3 4/10/18
* We certify that this work is entirely our own.  The assessor of this project may reproduce this project and provide copies to other academic staff, and/or communicate a copy of this project to a plagiarism-checking service, which may retain a copy of the project on its database.
*/

#include "a3_PhysicsWorld.h"

// include utilities on an as-needed basis
#include "animal3D/a3utility/a3_Timer.h"
#include "A3_DEMO\_utilities\a3_DemoShaderProgram.h"

// external
#include <stdio.h>
#include <string.h>


//-----------------------------------------------------------------------------
int setupBSP(BSP * bsp, a3real3p min, a3real3p max)
{
	a3real3Set(bsp->min.v, min[0], min[1], min[2]);
	a3real3Set(bsp->max.v, max[0], max[1], max[2]);

	//printf("Creating BSP with lower bounds (%lf %lf %lf) and upper bounds (%lf %lf %lf)\n", min[0], min[1], min[2], max[0], max[1], max[2]);
	return 0;
}


int setupBSPs(a3_PhysicsWorld * world, a3real3p min, a3real3p max, a3real3p units)
{
	a3vec3 boxUnits, tmp, tmpMin, tmpMax;

	// get the number of BSPs per dimension
	a3real3QuotientComp(boxUnits.v, a3real3Diff(tmp.v, max, min), units);

	int num = 0;
	for (int x = 0; x < (int)boxUnits.x; ++x)
	{
		for (int y = 0; y < (int)boxUnits.y; ++y)
		{
			for (int z = 0; z < (int)boxUnits.z; ++z)
			{
				++num;
				tmpMin.x = min[0] + x * units[0];
				tmpMin.y = min[1] + y * units[1];
				tmpMin.z = min[2] + z * units[2];

				a3real3Sum(tmpMax.v, tmpMin.v, units);

				setupBSP(world->bsps + num, tmpMin.v, tmpMax.v);
			}
		}
	}
	world->numBSPs = num;
	return 0;
}

int updateHulls(a3_PhysicsWorld * world)
{
	for (unsigned int j = 0; j < world->numBSPs; ++j)
	{
		world->bsps[j].numContainedHulls = 0;
	}

	for (unsigned int i = 0; i < world->rigidbodiesActive; ++i)
	{
		for (unsigned int j = 0; j < world->numBSPs; ++j)
		{
			if (world->hull[i].type == a3hullType_plane)
			{
				world->bsps[j].containedHulls[world->bsps[j].numContainedHulls] = world->hull + i;
				++world->bsps[j].numContainedHulls;
			}
			else if ((world->bsps[j].min.x <= (world->rigidbody + i)->position.x && world->bsps[j].max.x >= (world->rigidbody + i)->position.x) &&
				(world->bsps[j].min.y <= (world->rigidbody + i)->position.y && world->bsps[j].max.y >= (world->rigidbody + i)->position.y) &&
				(world->bsps[j].min.z <= (world->rigidbody + i)->position.z && world->bsps[j].max.z >= (world->rigidbody + i)->position.z))
			{
				world->bsps[j].containedHulls[world->bsps[j].numContainedHulls] = world->hull + i;
				++world->bsps[j].numContainedHulls;
				break;
			}
		}
	}
	return 0;
}



// internal utility for initializing and terminating physics world
void a3physicsInitialize_internal(a3_PhysicsWorld *world)
{
	//unsigned int i, j;

	// e.g. reset all particles and/or rigid bodies
	memset(world->rigidbody, 0, sizeof(world->rigidbody));
	memset(world->particle, 0, sizeof(world->particle));
	world->t = 0.0;

	// using random rotation
	a3randomSetSeed(0);


	// set up rigid bodies to test ray picking

	// ****TO-DO: 
	//	- add rotation to all

	// static shapes
	world->rigidbodiesActive = 0;

	const a3real PLANE_SIZE = 30.0f;

	world->rb_ground[0].position.x = a3realZero;
	world->rb_ground[0].position.y = a3realZero;
	world->rb_ground[0].position.z = -PLANE_SIZE;
	a3rigidbodySetMass(world->rb_ground, 0.0f);

	a3collisionCreateHullPlane(world->hull_ground + 0, world->rb_ground + 0, world->state->transform_rb + world->rigidbodiesActive, world->state->transformInv_rb + world->rigidbodiesActive,
		(a3real)(PLANE_SIZE), (a3real)(PLANE_SIZE), 1, a3axis_z);
	++world->rigidbodiesActive;

	world->rb_ground[1].position.x = a3realZero;
	world->rb_ground[1].position.y = a3realZero;
	world->rb_ground[1].position.z = PLANE_SIZE;

	a3rigidbodySetMass(world->rb_ground + 1, 0.0f);
	a3vec3 axis;
	axis.x = 0;
	axis.y = 1;
	axis.z = 0;

	a3quaternionCreateAxisAngle(world->state->rotation_rb[world->rigidbodiesActive].v, axis.v, 180.0f);
	a3collisionCreateHullPlane(world->hull_ground + world->rigidbodiesActive, world->rb_ground + world->rigidbodiesActive,
		world->state->transform_rb + world->rigidbodiesActive, world->state->transformInv_rb + world->rigidbodiesActive,
		(a3real)(PLANE_SIZE), (a3real)(PLANE_SIZE), 1, a3axis_z);
	++world->rigidbodiesActive;

	world->rb_ground[world->rigidbodiesActive].position.x = PLANE_SIZE;
	world->rb_ground[world->rigidbodiesActive].position.y = a3realZero;
	world->rb_ground[world->rigidbodiesActive].position.z = a3realZero;

	a3rigidbodySetMass(world->rb_ground + 1, 0.0f);
	axis.x = 0;
	axis.y = 1;
	axis.z = 0;

	a3quaternionCreateAxisAngle(world->state->rotation_rb[world->rigidbodiesActive].v, axis.v, 90.0f);
	a3collisionCreateHullPlane(world->hull_ground + world->rigidbodiesActive, world->rb_ground + world->rigidbodiesActive,
		world->state->transform_rb + world->rigidbodiesActive, world->state->transformInv_rb + world->rigidbodiesActive,
		(a3real)(PLANE_SIZE), (a3real)(PLANE_SIZE), 1, a3axis_z);
	++world->rigidbodiesActive;

	world->rb_ground[world->rigidbodiesActive].position.x = -PLANE_SIZE;
	world->rb_ground[world->rigidbodiesActive].position.y = a3realZero;
	world->rb_ground[world->rigidbodiesActive].position.z = a3realZero;

	a3rigidbodySetMass(world->rb_ground + 1, 0.0f);
	axis.x = 0;
	axis.y = 1;
	axis.z = 0;

	a3quaternionCreateAxisAngle(world->state->rotation_rb[world->rigidbodiesActive].v, axis.v, 270.0f);
	a3collisionCreateHullPlane(world->hull_ground + world->rigidbodiesActive, world->rb_ground + world->rigidbodiesActive,
		world->state->transform_rb + world->rigidbodiesActive, world->state->transformInv_rb + world->rigidbodiesActive,
		(a3real)(PLANE_SIZE), (a3real)(PLANE_SIZE), 1, a3axis_z);
	++world->rigidbodiesActive;

	world->rb_ground[world->rigidbodiesActive].position.x = a3realZero;
	world->rb_ground[world->rigidbodiesActive].position.y = PLANE_SIZE;
	world->rb_ground[world->rigidbodiesActive].position.z = a3realZero;

	a3rigidbodySetMass(world->rb_ground + 1, 0.0f);
	axis.x = 1;
	axis.y = 0;
	axis.z = 0;

	a3quaternionCreateAxisAngle(world->state->rotation_rb[world->rigidbodiesActive].v, axis.v, 270.0f);
	a3collisionCreateHullPlane(world->hull_ground + world->rigidbodiesActive, world->rb_ground + world->rigidbodiesActive,
		world->state->transform_rb + world->rigidbodiesActive, world->state->transformInv_rb + world->rigidbodiesActive,
		(a3real)(PLANE_SIZE), (a3real)(PLANE_SIZE), 1, a3axis_z);
	++world->rigidbodiesActive;

	world->rb_ground[world->rigidbodiesActive].position.x = a3realZero;
	world->rb_ground[world->rigidbodiesActive].position.y = -PLANE_SIZE;
	world->rb_ground[world->rigidbodiesActive].position.z = a3realZero;

	a3rigidbodySetMass(world->rb_ground + 1, 0.0f);
	axis.x = 1;
	axis.y = 0;
	axis.z = 0;

	a3quaternionCreateAxisAngle(world->state->rotation_rb[world->rigidbodiesActive].v, axis.v, 90.0f);
	a3collisionCreateHullPlane(world->hull_ground + world->rigidbodiesActive, world->rb_ground + world->rigidbodiesActive,
		world->state->transform_rb + world->rigidbodiesActive, world->state->transformInv_rb + world->rigidbodiesActive,
		(a3real)(PLANE_SIZE), (a3real)(PLANE_SIZE), 1, a3axis_z);
	++world->rigidbodiesActive;

	//world->rb_sphere[0].position.x = 0.0f;
	world->rb_sphere[0].position.y = -100.0f;
	world->rb_sphere[0].position.z = +5.0f;
	a3real3Set(world->rb_sphere[0].velocity.v, 0, 0, 0);
	a3rigidbodySetMass(world->rb_sphere, 0.5f);

	// moving shapes
	world->rb_sphere[1].position.x = -100.0f;
	world->rb_sphere[1].position.y = -10.0f;
	world->rb_sphere[1].position.z = +5.0f;
	world->rb_sphere[1].velocity.x = +15.0f;
	a3rigidbodySetMass(world->rb_sphere + 1, 0.5f);

	world->rb_sphere[2].position.x = -10.0f;
	world->rb_sphere[2].position.y = 0.0f;
	world->rb_sphere[2].position.z = +5.0f;
	world->rb_sphere[2].velocity.x = +20;
	a3rigidbodySetMass(world->rb_sphere + 2, 0.5f);

	world->rb_sphere[3].position.x = -10.0f;
	world->rb_sphere[3].position.y = +5.0f;
	world->rb_sphere[3].position.z = +10.0f;
	world->rb_sphere[3].velocity.x = +3;
	a3rigidbodySetMass(world->rb_sphere + 3, 0.5f);

	world->rb_sphere[4].position.x = -10.0f;
	world->rb_sphere[4].position.y = +10.0f;
	world->rb_sphere[4].position.z = +5.0f;
	a3rigidbodySetMass(world->rb_sphere + 4, 0.75f);

	for (int i = 0; i < 5; ++i, ++world->rigidbodiesActive)
		a3collisionCreateHullSphere(world->hull_sphere + i, world->rb_sphere + i, world->state->transform_rb + world->rigidbodiesActive,
			world->state->transformInv_rb + world->rigidbodiesActive,
			a3randomRange(a3realHalf, a3realTwo));


	// no particles today
	world->particlesActive = 0;

	// raise initialized flag
	world->init = 1;
	a3vec3 min, max, units;
	a3real3Set(min.v, -100, -100, -100);
	a3real3Set(max.v, 100, 100, 100);
	a3real3Set(units.v, 100, 100, 100);

	setupBSPs(world, min.v, max.v, units.v);

	world->framesSkipped = 0;

	// Create the compute program
	// Set the program to equal negative one
	// In update - conditional check - if program = -1, don't do it, otherwise do it
//	world->computeShader = -1;


	wglMakeCurrent(*world->dcRef, *world->physicsRenderContext);

	// list of all unique shaders
	// this is a good idea to avoid multi-loading 
	//	those that are shared between programs
	union {
		struct {
			// vertex shaders
			a3_Shader passLambertComponents_eye_transform_vs[1];
			a3_Shader passTexcoord_transform_vs[1];
			a3_Shader passthru_transform_vs[1];
			a3_Shader passColor_transform_vs[1];
			a3_Shader dummy_vs[1];

			// geometry shaders
			a3_Shader drawRay_gs[1];

			// fragment shaders
			a3_Shader drawLambert_eye_fs[1];
			a3_Shader drawColorUnifTexture_fs[1];
			a3_Shader drawColorUnif_fs[1];
			a3_Shader drawColorAttrib_fs[1];

			// compute shaders
			//TYLER GO
			a3_Shader physicsCompute_cs[1];
		};
	} shaderList = { 0 };
	a3_Shader *const shaderListPtr = (a3_Shader *)(&shaderList);

	//TYLER GO
	struct {
		a3_ShaderType shaderType;
		unsigned int srcCount;
		const char *filePath[8];	// max number of source files per shader
	} shaderDescriptor[] = {
		{ a3shader_vertex,		1,{ "../../../../resource/glsl/4x/vs/e/passLambertComponents_eye_transform_vs4x.glsl" } },
	{ a3shader_vertex,		1,{ "../../../../resource/glsl/4x/vs/e/passTexcoord_transform_vs4x.glsl" } },
	{ a3shader_vertex,		1,{ "../../../../resource/glsl/4x/vs/e/passthru_transform_vs4x.glsl" } },
	{ a3shader_vertex,		1,{ "../../../../resource/glsl/4x/vs/e/passColor_transform_vs4x.glsl" } },
	{ a3shader_vertex,		1,{ "../../../../resource/glsl/4x/vs/e/dummy_vs4x.glsl" } },

	{ a3shader_geometry,	1,{ "../../../../resource/glsl/4x/gs/e/drawRay_gs4x.glsl" } },

	{ a3shader_fragment,	1,{ "../../../../resource/glsl/4x/fs/e/drawLambert_eye_fs4x.glsl" } },
	{ a3shader_fragment,	1,{ "../../../../resource/glsl/4x/fs/e/drawColorUnifTexture_fs4x.glsl" } },
	{ a3shader_fragment,	1,{ "../../../../resource/glsl/4x/fs/e/drawColorUnif_fs4x.glsl" } },
	{ a3shader_fragment,	1,{ "../../../../resource/glsl/4x/fs/e/drawColorAttrib_fs4x.glsl" } },
	//TYLER GO
	{ a3shader_compute,		1,{ "../../../../resource/glsl/4x/cs/physicsCompute_header.glsl" } },
	};

	a3shaderCreateFromFileList(shaderListPtr + 10, shaderDescriptor[10].shaderType,
		shaderDescriptor[10].filePath, shaderDescriptor[10].srcCount);

	a3shaderProgramCreate(world->computeShader->program);
	a3shaderProgramAttachShader(world->computeShader->program, shaderList.physicsCompute_cs);

	a3shaderProgramLink(world->computeShader->program);
	a3shaderProgramValidate(world->computeShader->program);

	// if linking fails, contingency plan goes here
	// otherwise, release shaders
	//a3shaderRelease(shaderList.physicsCompute_cs);

	// bind ssbo buffer
	mRigidBodyCount = 11;
	ssboBindBuffer(&world->ssboCount, sizeof(unsigned int), &mRigidBodyCount, 1);
	ssboBindBuffer(&world->ssboVelocities, sizeof(a3vec4) * mRigidBodyCount, &velocities, 2);
	ssboBindBuffer(&world->ssboPositions, sizeof(a3vec4) * mRigidBodyCount, &positions, 3);
	ssboBindBuffer(&world->ssboMassInv, sizeof(float) * mRigidBodyCount, &massInvs, 4);
	ssboBindBuffer(&world->ssboType, sizeof(unsigned int) * mRigidBodyCount, &types, 5);
	ssboBindBuffer(&world->ssboCharOne, sizeof(unsigned int) * mRigidBodyCount, &characteristicOnes, 6);
	ssboBindBuffer(&world->ssboCharTwo, sizeof(unsigned int) * mRigidBodyCount, &characteristicTwos, 7);
	ssboBindBuffer(&world->ssboCharThree, sizeof(unsigned int) * mRigidBodyCount, &characteristicThrees, 8);
	ssboBindBuffer(&world->ssboCharFour, sizeof(unsigned int) * mRigidBodyCount, &characteristicFours, 9);

	ssboWriteBuffer(world->ssboCount, sizeof(unsigned int), &mRigidBodyCount);
	ssboWriteBuffer(world->ssboVelocities, sizeof(a3vec4) * mRigidBodyCount, &velocities);
	ssboWriteBuffer(world->ssboPositions, sizeof(a3vec4) * mRigidBodyCount, &positions);
	ssboWriteBuffer(world->ssboMassInv, sizeof(float) * mRigidBodyCount, &massInvs);
	ssboWriteBuffer(world->ssboType, sizeof(unsigned int) * mRigidBodyCount, &types);
	ssboWriteBuffer(world->ssboCharOne, sizeof(unsigned int) * mRigidBodyCount, &characteristicOnes);
	ssboWriteBuffer(world->ssboCharTwo, sizeof(unsigned int) * mRigidBodyCount, &characteristicTwos);
	ssboWriteBuffer(world->ssboCharThree, sizeof(unsigned int) * mRigidBodyCount, &characteristicThrees);
	ssboWriteBuffer(world->ssboCharFour, sizeof(unsigned int) * mRigidBodyCount, &characteristicFours);

	// i hate this but it works
	for (int i = 0; i < 11; ++i)
	{
		if ((world->hull + i)->type == a3hullType_sphere)
			a3real3Set((world->rigidbody + i)->velocity.v, a3realZero, a3realZero, 3);
	}

	// reset state
	a3physicsWorldStateReset(world->state);
}

void a3physicsTerminate_internal(a3_PhysicsWorld *world)
{
	// any term tasks here
}


//-----------------------------------------------------------------------------

void a3handleCollision(a3_ConvexHullCollision* collision, a3_ConvexHull* hull_a, a3_ConvexHull* hull_b)
{
	// http://www.chrishecker.com/images/e/e7/Gdmphys3.pdf

	a3vec3 rVel;
	a3real3Diff(rVel.v, hull_a->rb->velocity.v, hull_b->rb->velocity.v);

	a3real j1 = (-a3realTwo * a3real3Dot(rVel.v, collision->normal_a[0].v)) / (a3real3Dot(collision->normal_a[0].v,
		collision->normal_a[0].v)*(hull_a->rb->massInv + hull_b->rb->massInv));

	a3real3Add(hull_a->rb->velocity.v, a3real3ProductS(rVel.v, collision->normal_a[0].v, (j1 * hull_a->rb->massInv)));
	a3real3Add(hull_b->rb->velocity.v, a3real3ProductS(rVel.v, collision->normal_b[0].v, (j1 * hull_b->rb->massInv)));
}

// physics simulation
void a3physicsUpdate(a3_PhysicsWorld *world, double dt)
{
	// copy of state to edit before writing to world
	a3_PhysicsWorldState state[1] = { 0 };

	// time as real
	const a3real t_r = (a3real)(world->t);
	const a3real dt_r = (a3real)(dt);

	// generic counter
	unsigned int i;

	// ****TO-DO: 
	//	- write to state
	for (i = 0; i < world->rigidbodiesActive; ++i)
	{
		state->position_rb[i].xyz = world->rigidbody[i].position;
		state->rotation_rb[i] = world->state->rotation_rb[i];

		// rotation
		a3quaternionConvertToMat4(state->transform_rb[i].m, state->rotation_rb[i].v, state->position_rb[i].v);
		a3real4x4TransformInverseIgnoreScale(state->transformInv_rb[i].m, state->transform_rb[i].m);
	}


	//a3vec3 tmp;

	/*for (int i = 0; i < 5; i++)
	{
		a3forceGravity(tmp.v, a3zVec3.v, world->hull_sphere[i].rb->mass);
		a3real3Add(world->hull_sphere[i].rb->force.v, tmp.v);
		a3real3Add(world->hull_sphere[i].rb->force.v,
			a3forceDrag(tmp.v, world->hull_sphere[i].rb->velocity.v, a3zeroVec3.v, 1.2f, world->hull_sphere[i].prop[a3hullProperty_radius], .47f));
	}*/

	state->count_rb = i;
	for (i = 0; i < world->particlesActive; ++i)
	{
		a3real4SetReal3W(state->position_p[i].v, world->particle[i].position.v, a3realOne);
	}
	state->count_p = i;

	// accumulate time
	world->t += dt;

	if (world->framesSkipped >= 5)
	{
		wglMakeCurrent(*world->dcRef, *world->physicsRenderContext);
		//---------------------------------------------------------------------
		// DO THIS AFTER INTEGRATION
		// write to the buffer, current rigidbody data
		mRigidBodyCount = world->rigidbodiesActive;
		for (unsigned int i = 0; i < mRigidBodyCount; ++i)
		{
			a3real4Set(velocities[i].v, world->rigidbody[i].velocity.x, world->rigidbody[i].velocity.y, world->rigidbody[i].velocity.z, 1);
			a3real4Set(positions[i].v, world->rigidbody[i].position.x, world->rigidbody[i].position.y, world->rigidbody[i].position.z, 1);
			massInvs[i] = world->rigidbody[i].massInv;

			types[i] = world->hull[i].type == a3hullType_sphere ? 0 : 1;
			characteristicOnes[i] = world->hull[i].type == a3hullType_sphere ? (unsigned int)world->hull[i].prop[a3hullProperty_radius] : (unsigned int)world->hull[i].prop[a3hullProperty_halfwidth];
			characteristicTwos[i] = world->hull[i].type == a3hullType_sphere ? (unsigned int)world->hull[i].prop[a3hullProperty_radius] : (unsigned int)world->hull[i].prop[a3hullProperty_halfheight];
			characteristicThrees[i] = world->hull[i].type == a3hullType_sphere ? (unsigned int)world->hull[i].prop[a3hullProperty_radius]: (unsigned int)world->hull[i].prop[a3hullProperty_halfdepth];
		}

		ssboWriteBuffer(world->ssboCount, sizeof(unsigned int), &mRigidBodyCount);
		ssboWriteBuffer(world->ssboVelocities, sizeof(a3vec4) * mRigidBodyCount, &velocities);
		ssboWriteBuffer(world->ssboPositions, sizeof(a3vec4) * mRigidBodyCount, &positions);
		ssboWriteBuffer(world->ssboMassInv, sizeof(float) * mRigidBodyCount, &massInvs);
		ssboWriteBuffer(world->ssboType, sizeof(unsigned int) * mRigidBodyCount, &types);
		ssboWriteBuffer(world->ssboCharOne, sizeof(unsigned int) * mRigidBodyCount, &characteristicOnes);
		ssboWriteBuffer(world->ssboCharTwo, sizeof(unsigned int) * mRigidBodyCount, &characteristicTwos);
		ssboWriteBuffer(world->ssboCharThree, sizeof(unsigned int) * mRigidBodyCount, &characteristicThrees);
		ssboWriteBuffer(world->ssboCharFour, sizeof(unsigned int) * mRigidBodyCount, &characteristicFours);

		//
		// dispatch the compute shader
		//TYLER GO
		//if (world->computeShader != -1)
		{
			a3shaderProgramActivate(world->computeShader->program);
			glDispatchCompute(11, 1, 1);
		}
		//
		// read the data back

		ssboReadBuffer(world->ssboVelocities, sizeof(a3vec4) * mRigidBodyCount, &velocities);

		for (unsigned int i = 0; i < mRigidBodyCount; ++i)
		{
			a3real3Set(world->rigidbody[i].velocity.v, velocities[i].x, velocities[i].y, velocities[i].z);
			//printf("%lf\n", velocities[i].x);
		}

		printf("GPU out: %lf %lf %lf\n", velocities[7].x, velocities[7].y, velocities[7].z);
		printf("CPU val: %lf %lf %lf\n", world->rigidbody[7].velocity.x, world->rigidbody[7].velocity.y, world->rigidbody[7].velocity.z);

		// ****TO-DO: 
		//	- apply forces and torques

		for (i = 1; i < world->rigidbodiesActive; ++i)
		{
			a3rigidbodyIntegrateEulerKinematic(world->rigidbody + i, dt_r);
			a3real3ProductS(world->rigidbody[i].acceleration.v, world->rigidbody[i].force.v, world->rigidbody[i].massInv);
			//Add set to acceleration
			a3real4ProductS(world->rigidbody[i].acceleration_a.v, world->rigidbody[i].torque.v, world->rigidbody[i].massInv);
			a3real4Normalize(world->rigidbody[i].acceleration_a.v);

			//reset force
			a3rigidbodyResetForce(world->rigidbody + i);
		}
		for (i = 0; i < world->particlesActive; ++i)
		{
			a3particleIntegrateEulerSemiImplicit(world->particle + i, dt_r);
		}

		//---------------------------------------------------------------------
		a3shaderProgramDeactivate(world->computeShader->program);
	}
	else
		world->framesSkipped++;

	// write operation is locked
	if (a3physicsLockWorld(world) > 0)
	{
		// copy state to world
		*world->state = *state;
		a3physicsUnlockWorld(world);
	}
}

// physics thread
long a3physicsThread(a3_PhysicsWorld *world)
{
	// physics simulation timer
	a3_Timer physicsTimer[1] = { 0 };

	// second counter for physics (debugging)
	unsigned int currSecond = 0, prevSecond = 0;

	// create world
	a3physicsInitialize_internal(world);

	// start timer
	// rate should be set before beginning thread
	a3timerSet(physicsTimer, world->rate);
	a3timerStart(physicsTimer);

	// if lock is negative, terminate
	while (world->lock >= 0)
	{
		if (a3timerUpdate(physicsTimer))
		{
			// update timer ticked, do the thing
			a3physicsUpdate(world, physicsTimer->previousTick);

			// debug display time in seconds
			currSecond = (unsigned int)(physicsTimer->totalTime);
			if (currSecond > prevSecond)
			{
				prevSecond = currSecond;
			}
		}
	}

	// terminate world
	a3physicsTerminate_internal(world);
	return 0;
}


//-----------------------------------------------------------------------------

// reset world state
int a3physicsWorldStateReset(a3_PhysicsWorldState *worldState)
{
	unsigned int i;
	if (worldState)
	{
		//	- reset all state data appropriately
		for (i = 0; i < physicsMaxCount_rigidbody; ++i)
		{
			worldState->position_rb[i] = a3wVec4;
			//worldState->rotation_rb[i] = a3wVec4;
			worldState->transform_rb[i] = a3identityMat4;
			worldState->transformInv_rb[i] = a3identityMat4;
		}
		for (i = 0; i < physicsMaxCount_particle; ++i)
		{
			worldState->position_p[i] = a3wVec4;
		}
		return (physicsMaxCount_rigidbody + physicsMaxCount_particle);
	}
	return -1;
}


//-----------------------------------------------------------------------------

// get thread ID
#ifdef _WIN32
#include <Windows.h>
#include <stdlib.h>
int threadID()
{
	return GetCurrentThreadId();
}
#else
#include <sys/types.h>
int threadID()
{
	return gettid();
}
#endif	// _WIN32

// mutex
extern inline int a3physicsLockWorld(a3_PhysicsWorld *world)
{
	// wait for lock to be released, then set it
	while (world->lock > 0);
	if (world->lock == 0)
	{
		world->lock = threadID();
		return world->lock;
	}
	return -1;
}

extern inline int a3physicsUnlockWorld(a3_PhysicsWorld *world)
{
	const int ret = world->lock;
	if (ret == threadID())
	{
		world->lock = 0;
		return ret;
	}
	return -1;
}

void ssboBindBuffer(GLuint* program, GLuint dataSize, void *bufferData, GLuint bindingLocation)
{
	glGenBuffers(1, program);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, *program);
	glBufferData(GL_SHADER_STORAGE_BUFFER, dataSize, bufferData, GL_DYNAMIC_COPY);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingLocation, *program);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

//-----------------------------------------------------------------------------
// Final project functions
void ssboWriteBuffer(GLuint program, GLuint dataSize, void *data)
{
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, program);
	GLvoid* ptr = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(ptr, data, dataSize);
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}

void ssboReadBuffer(GLuint program, GLuint dataSize, void *dest)
{
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, program);
	GLvoid* ptr = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE);
	memcpy(dest, ptr, dataSize);
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}