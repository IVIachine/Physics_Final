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
	
	a3_PhysicsWorld.h
	Interface for physics world.
*/

/*
* IDs: 0955181 and 0967813
* EGP 425-01 Project 3 4/10/18
* We certify that this work is entirely our own.  The assessor of this project may reproduce this project and provide copies to other academic staff, and/or communicate a copy of this project to a plagiarism-checking service, which may retain a copy of the project on its database.
*/

#ifndef __ANIMAL3D_PHYSICSWORLD_H
#define __ANIMAL3D_PHYSICSWORLD_H


//-----------------------------------------------------------------------------
// physics includes
#include "a3_Collision.h"
#include "animal3D\a3graphics\a3_ShaderProgram.h"
#include "A3_DEMO\_utilities\a3_DemoShaderProgram.h"
#include "../BSP.h"
#include "GL/glew.h"
#include <Windows.h>
//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
	typedef struct a3_PhysicsWorld					a3_PhysicsWorld;
	typedef struct a3_PhysicsWorldState				a3_PhysicsWorldState;
	typedef struct a3_DemoStateShaderProgram		a3_DemoStateShaderProgram;
	typedef struct rigidBodySSBOBufferData			rigidBodySSBOBufferData;
	typedef struct fakeRigidBody					fakeRigidBody;
#endif	// __cplusplus


//-----------------------------------------------------------------------------

	// counters
	enum a3_PhysicsWorldMaxCount
	{
		physicsMaxCount_particle = 32,
		physicsMaxCount_rigidbody = 32,
		physicsMaxCount_bsp = 32,
	};


//-----------------------------------------------------------------------------

	// state of a physics world: things that can be used for graphics ONLY
	//	- position and rotation... why not scale? RIGID bodies don't scale
	struct a3_PhysicsWorldState
	{
		//	- add particle position
		a3vec4 position_p[physicsMaxCount_particle];

		//	- add rigid body position and rotation
		//	- add matrix for solving inertia tensor
		//		-> we will use this and its inverse (transpose) 
		//			to resolve inertia tensors
		a3vec4 position_rb[physicsMaxCount_rigidbody];
		a3vec4 rotation_rb[physicsMaxCount_rigidbody];
		a3mat4 transform_rb[physicsMaxCount_rigidbody];
		a3mat4 transformInv_rb[physicsMaxCount_rigidbody];

		// current counts
		unsigned int count_p, count_rb;
	};


//-----------------------------------------------------------------------------
	
//-----------------------------------------------------------------------------

	// threaded simulation
	void a3physicsUpdate(a3_PhysicsWorld *world, double dt);
	long a3physicsThread(a3_PhysicsWorld *world);

	// world utilities
	int a3physicsWorldStateReset(a3_PhysicsWorldState *worldState);

	// mutex handling
	inline int a3physicsLockWorld(a3_PhysicsWorld *world);
	inline int a3physicsUnlockWorld(a3_PhysicsWorld *world);


//-----------------------------------------------------------------------------
// For the final project
// We need:
// a buffer that stores all the rigidbodies
// a max number of rigidbodies
	struct fakeRigidBody
	{
		a3vec4 velocity;
		a3vec4 position;
		float massInv;

		unsigned int type;
		unsigned int characteristicOne, characteristicTwo, characteristicThree, characteristicFour;
	};

	struct rigidBodySSBOBufferData
	{
		unsigned int rigidBodyCount;
		fakeRigidBody rigidbody[physicsMaxCount_rigidbody];
	};

		// persistent physics world data structure
	struct a3_PhysicsWorld
	{
		//---------------------------------------------------------------------
		// SIMPLE MUTEX LOCK
		int lock;


		//---------------------------------------------------------------------
		// initialized flag
		int init;
		int framesSkipped;

		//---------------------------------------------------------------------
		// timer rate
		double rate;


		//---------------------------------------------------------------------
		// the state to store all of the things that need to go to graphics
		a3_PhysicsWorldState state[1];

		//---------------------------------------------------------------------

		//---------------------------------------------------------------------
		// general variables pertinent to the state

		// e.g. set of rigid bodies
		union {
			a3_RigidBody rigidbody[physicsMaxCount_rigidbody];
			struct {
				a3_RigidBody rb_ground[6];
				a3_RigidBody rb_sphere[5];
			};
		};
		unsigned int rigidbodiesActive;
		
		// e.g. set of particles
		union {
			a3_Particle particle[physicsMaxCount_particle];
		};
		unsigned int particlesActive;

		// e.g. set of hulls for rigid bodies
		union {
			a3_ConvexHull hull[physicsMaxCount_rigidbody];
			struct {
				a3_ConvexHull hull_ground[6];
				a3_ConvexHull hull_sphere[5];
			};
		};

		double t;

		BSP bsps[physicsMaxCount_bsp];
		unsigned int numBSPs;
		//---------------------------------------------------------------------
		// SSBO handles
		GLuint ssboRigidbodies;
		HGLRC physicsRenderContext[1];
		HDC dcRef[1];
		a3_DemoStateShaderProgram computeShader[1];
	};

	rigidBodySSBOBufferData sendMetoTheShader;

	void ssboBindBuffer(GLuint* program, GLuint dataSize, void *bufferData, GLuint bindingLocation);
// we need access to openGL to bind the buffers for the SSBOs
	void ssboWriteBuffer(GLuint program, GLuint dataSize, void *data);
	void ssboReadBuffer(GLuint program, GLuint dataSize, void *dest);

	// how to use
	//setup compute program (need to make a function for it)
	//bind the buffer
	//initialize the buffer data
	//write to the ssbos before calculations
	//  call the compute program (glUseProgram)
	//  calculate changes to rigidbodies
	//write output to physics world rigidbodies

//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// !__ANIMAL3D_PHYSICSWORLD_H