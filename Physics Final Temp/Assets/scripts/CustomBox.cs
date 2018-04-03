using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
struct BoxColliderRigidbody
{
    public float minX, maxX, minY, maxY, minZ, maxZ;
    public Matrix4x4 transformInv, transform;

    public Vector3 velocity, acceleration, force;
}

public class CustomParticle
{
    private BoxColliderRigidbody mBoxCollider;

    public CustomParticle(Bounds b, Transform transformRef)
    {
        mBoxCollider = new BoxColliderRigidbody();
        mBoxCollider.minX = b.min.x;
        mBoxCollider.minY = b.min.y;
        mBoxCollider.minZ = b.min.z;

        mBoxCollider.maxX = b.max.x;
        mBoxCollider.maxY = b.max.y;
        mBoxCollider.maxZ = b.max.z;

        mBoxCollider.velocity = mBoxCollider.acceleration = mBoxCollider.force = Vector3.zero;

        //set the transform and transform inv
        mBoxCollider.transform = transformRef.localToWorldMatrix;
        mBoxCollider.transformInv = transformRef.worldToLocalMatrix;
    }
}