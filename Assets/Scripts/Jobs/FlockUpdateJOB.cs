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
        [ReadOnly] public NativeArray<float3> _OldPos;
        [ReadOnly] public NativeArray<float3> _OldDir;
        [ReadOnly] public NativeArray<float> _OldVel;

        [NativeDisableParallelForRestriction] public NativeArray<float3> _Dir;
        [NativeDisableParallelForRestriction] public NativeArray<float> _Vel;

        [ReadOnly] public NativeMultiHashMap<int, int> _GridToBoidsMap;

        [ReadOnly] public NativeArray<RaycastHit> _RayCastHits;

        public Unity.Mathematics.Random rand;
        public float deltaTime, changeVelocity;
        public float separationRad, cohesionRad;
        public float maxVel, minVel;

        //Index goes trought every box
        public void Execute(int index)
        {
            if (_GridToBoidsMap.ContainsKey(index))
            {
                //! Calculate local values
                float3 localPosition = float3.zero, localDirection = float3.zero;
                float localVelocity = 0;
                var iterator = _GridToBoidsMap.GetValuesForKey(index);
                int counter = 0;

                do
                {
                    int boidID = iterator.Current;

                    localPosition += _OldPos[boidID];
                    localDirection += _OldDir[boidID];
                    localVelocity += _OldVel[boidID];

                    counter++;

                } while (iterator.MoveNext());

                localPosition *= 1.0f / counter;
                localDirection *= 1.0f / counter;
                localVelocity *= 1.0f / counter;

                iterator.Reset();

                //! Do the work
                do
                {
                    int boidID = iterator.Current;

                    float t = deltaTime * changeVelocity;
                    float3 separation =localPosition - _OldPos[boidID];
                    float dsq = separation.x * separation.x + separation.y * separation.y + separation.z * separation.z;

                    //! New direction
                    var newDir = math.normalize(Vector3.Lerp(_OldDir[boidID], localDirection, t));
                    
                    if (_RayCastHits[boidID].normal.x * _RayCastHits[boidID].normal.y * _RayCastHits[boidID].normal.z != 0)
                    {
                        newDir = math.normalize(Vector3.LerpUnclamped(newDir, _RayCastHits[boidID].normal, deltaTime * 1.0f/ _RayCastHits[boidID].distance));
                        //newDir = _RayCastHits[boidID].normal;
                    }

                    //newDir += rand.NextFloat3Direction() * rand.NextFloat(0.0f, 0.1f);
                    _Dir[boidID] = newDir;

                    //! New Velocity
                    _Vel[boidID] = math.lerp(_OldVel[boidID], localVelocity, t);

                    //! If it is too close
                    if (dsq <= separationRad)
                    {
                        if (_Vel[boidID] > localVelocity)
                            _Vel[boidID] -= changeVelocity * deltaTime;

                    }
                    else if (dsq > cohesionRad)    //! If it is too far
                    {
                        if (_Vel[boidID] <= localVelocity)
                            _Vel[boidID] += changeVelocity * deltaTime;
                    }

                    _Vel[boidID] = math.clamp(_Vel[boidID], minVel, 1);

                    

                } while (iterator.MoveNext());
            }
        }
    }
}