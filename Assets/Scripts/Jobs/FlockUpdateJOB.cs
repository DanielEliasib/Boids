using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    //Flock Behavior
    public struct FlockUpdateJOB : IJobParallelFor
    {
        //! LookUp Data
        [ReadOnly] public NativeArray<float3> _OldPos;
        [ReadOnly] public NativeArray<float3> _OldVel; 

        //! New Generated Data
        //: Be carefull this array already has data from Obstacle evasion
        [NativeDisableParallelForRestriction] public NativeArray<float3> _NewVel;

        //! Simulation Area Data
        [ReadOnly] public NativeParallelMultiHashMap<int, int> _GridToBoidsMap;
        [ReadOnly] public SimulationArea _SimArea;

        //! Obstacle Data
        [ReadOnly] public NativeArray<RaycastHit> _RayCastHits;
        [ReadOnly] public float _VisDistance;
        [ReadOnly] public BoidSystemOptions _SystemOptions;

        //! Other
        public Unity.Mathematics.Random rand;
        public float deltaTime;

        //: Index goes trought every box
        public void Execute(int index)
        {
            if (_GridToBoidsMap.ContainsKey(index))
            {
                //  ********************************
                //: Calculate local values
                //? Should this be done in another JOB?
                float3 localPosition = float3.zero, localVelocity = float3.zero;

                var iterator = _GridToBoidsMap.GetValuesForKey(index);
                int counter = 0;

                while (iterator.MoveNext())
                {
                    int boidID = iterator.Current;

                    localPosition += _OldPos[boidID];
                    localVelocity += _OldVel[boidID];

                    counter++;
                }

                iterator.Reset();

                if(counter > 1)
                {
                    //  ********************************
                    //! Do the work
                    while (iterator.MoveNext())
                    {
                        int boidID = iterator.Current;
                        float3 realLocalPosition = (localPosition - _OldPos[boidID])/(counter-1);
                        float3 realLocalVelocity = (localVelocity - _OldVel[boidID])/(counter-1);
                        
                        float t = deltaTime * _SystemOptions.ChangeRate;

                        //: New direction
                        //? Is Lerp fast enogh?
                        float3 newVelocity = float3.zero;
                        newVelocity += realLocalVelocity;

                        //? How to make it so the randomness only goes in the direction it's already moving.
                        //newVelocity += rand.NextFloat3Direction() * rand.NextFloat(0.0f, _SystemOptions.NoiseMagnitude);

                        //: New Speed
                        float3 boidDir = math.normalize(_OldVel[boidID]);
                        float3 separation = realLocalPosition - _OldPos[boidID];
                        float dsq = separation.x * separation.x + separation.y * separation.y + separation.z * separation.z;

                        float speedSqr = _OldVel[boidID].x * _OldVel[boidID].x + _OldVel[boidID].y * _OldVel[boidID].y + _OldVel[boidID].z * _OldVel[boidID].z;
                        float localSpeedSqr = realLocalVelocity.x * realLocalVelocity.x + realLocalVelocity.y * realLocalVelocity.y + realLocalVelocity.z * realLocalVelocity.z;

                        //! If it is too close
                        if (dsq <= _SystemOptions.SeparationRadius)
                        {
                            if (speedSqr >= localSpeedSqr)
                                newVelocity -= boidDir * t;

                        }
                        else if (dsq > _SystemOptions.CohesionRadius)    //! If it is too far
                        {
                            if (speedSqr <= localSpeedSqr)
                                newVelocity += boidDir * _SystemOptions.ChangeRate * t;
                        }

                        _NewVel[boidID] += newVelocity;
                    }
                }
            }
        }

    }
    
}