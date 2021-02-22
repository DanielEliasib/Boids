using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using UnityEngine;

namespace AL.BoidSystem.Jobs
{
    public struct CollisionForceJOB : IJobParallelFor
    {
        //! This will be shared through several JOB. Is it thread safe?
        public NativeArray<float3> _CorrectionForce;

        [ReadOnly] public NativeArray<RaycastHit> _RayCastHits; //It contains all the hits from all the boids.                      lenght = nOfBoids * nOfRays
        [ReadOnly] public NativeArray<float3> _RayDirections;   //It contains the local directions for the raycast fo one boid.     lenght = nOfRays
        [ReadOnly] public NativeArray<float4x4> _TransMatrices;    //One matrix per boid.                                          lenght = nOfBoids
        [ReadOnly] public NativeArray<float3> _OldVelocity;     //The velocities for every boid.                                    lenght = nOfBoids

        [ReadOnly] public int _NumberOfBoids;
        [ReadOnly] public int _NumberOfRays;

        //: Index goes through every boid
        //! y * width + x

        public void Execute(int boidID)
        {
            float3 collisionForce = float3.zero;

            //! Get the first ray
            int realIndex = 0 * _NumberOfBoids + boidID;

            float3 normal = _RayCastHits[realIndex].normal;

            bool freeFoud = false;
            bool collision = false;

            //If normal != float3.zero it means it hited something.
            if (math.abs(normal.x + normal.y + normal.z) > 4 * float.Epsilon)
            {
                //Mark if as collision.
                collision = true;

                //Check all the other rays until you find one that doesn't collide.
                for (int i = 1; i < _NumberOfRays; i++)
                {
                    realIndex += _NumberOfBoids;
                    normal = _RayCastHits[realIndex].normal;
                    if (math.abs(normal.x + normal.y + normal.z) <= 4 * float.Epsilon)
                    {
                        
                        collisionForce = math.mul(_TransMatrices[boidID], new float4(_RayDirections[i], 0)).xyz*2.5f - _OldVelocity[boidID];
                        freeFoud = true;
                        break;
                    }
                }
            }

            if (freeFoud)
                _CorrectionForce[boidID] += collisionForce;
            else if (collision)
                _CorrectionForce[boidID] += normal;
        }
    }
}