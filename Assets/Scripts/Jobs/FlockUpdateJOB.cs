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
        [ReadOnly] public NativeArray<float3> _OldVel;  // Let's make it so Dirs has also velocity information

        //! New Generated Data
        [NativeDisableParallelForRestriction] public NativeArray<float3> _Vel;

        //! Simulation Area Data
        [ReadOnly] public NativeMultiHashMap<int, int> _GridToBoidsMap;
        [ReadOnly] public SimulationArea _SimArea;

        //! Obstacle Data
        [ReadOnly] public NativeArray<RaycastHit> _RayCastHits;
        [ReadOnly] public float _VisDistance;
        [ReadOnly] public BoidSystemOptions _SystemOptions;

        //! Other
        public Unity.Mathematics.Random rand;
        public float deltaTime;

        //Index goes trought every box
        public void Execute(int index)
        {
            if (_GridToBoidsMap.ContainsKey(index))
            {
                //  ********************************
                //: Calculate local values
                //? Should this be done in other JOB?
                float3 localPosition = float3.zero, localVelocity = float3.zero;

                var iterator = _GridToBoidsMap.GetValuesForKey(index);
                int counter = 0;

                do
                {
                    int boidID = iterator.Current;

                    localPosition += _OldPos[boidID];
                    localVelocity += _OldVel[boidID];

                    counter++;

                } while (iterator.MoveNext());

                localPosition *= 1.0f / counter;    //! Counter should be at least one.
                localVelocity *= 1.0f / counter;

                iterator.Reset();

                //  ********************************
                //! Do the work
                do
                {
                    int boidID = iterator.Current;
                    float t = deltaTime * _SystemOptions.ChangeRate;

                    //: New direction
                    //? Is Lerp fast enogh?
                    //float3 newVelocity = _OldVel[boidID];
                    float3 newVelocity = Vector3.Lerp(_OldVel[boidID], localVelocity, t);  //Steers towards the avarage direction but also keeps it's magnitude which is the speed.

                    //: Avoid Collisions
                    if (_RayCastHits[boidID].normal.x * _RayCastHits[boidID].normal.y * _RayCastHits[boidID].normal.z != 0)
                    {
                        newVelocity = Vector3.LerpUnclamped(newVelocity, _RayCastHits[boidID].normal, t / _RayCastHits[boidID].distance);   //The closer it get's 1/d tends to infinity.
                    }

                    //: Avoid Simulation Walls
                    float3 halfArea = _SimArea.Size * 0.5f;
                    float3 boidDir = Vector3.Normalize(_OldVel[boidID]);    //? Is normalize worth it?

                    float3 intersectionParameters = halfArea - math.abs(_OldPos[boidID]);
                    intersectionParameters = intersectionParameters / (_VisDistance * boidDir);
                    
                    var closestIntersec = minValue(ref intersectionParameters);

                    // 0 -> zy plane
                    // 1 -> xz plane
                    // 2 -> yz plane
                    float3 normal = float3.zero;
                    float3 signs;

                    if (closestIntersec.val <= 1.0f)    //! If it's grather than one then it is too far away of behind.
                    {
                        float3 interPoint = _OldPos[boidID] + _VisDistance * boidDir * closestIntersec.val;
                        switch (closestIntersec.index)
                        {
                            case 0:
                                normal = new float3(-1.0f, 0.0f, 0.0f);
                                signs = math.sign(interPoint);
                                normal = normal * signs;
                                
                            break;
                            case 1:
                                normal = new float3(0.0f, -1.0f, 0.0f);
                                signs = math.sign(interPoint);
                                normal = normal * signs;
                                break;
                            case 2:
                                normal = new float3(0.0f, 0.0f, -1.0f);
                                signs = math.sign(interPoint);
                                normal = normal * signs;
                                break;
                            default:
                                break;
                        }

                        newVelocity = Vector3.LerpUnclamped(newVelocity, normal, deltaTime / closestIntersec.val);
                    }

                    newVelocity += rand.NextFloat3Direction() * rand.NextFloat(0.0f, _SystemOptions.NoiseMagnitude);

                    //: New Speed
                    //float3 separation = localPosition - _OldPos[boidID];
                    //float dsq = separation.x * separation.x + separation.y * separation.y + separation.z * separation.z;

                    //float speedSqr = _OldVel[boidID].x * _OldVel[boidID].x + _OldVel[boidID].y * _OldVel[boidID].y + _OldVel[boidID].z * _OldVel[boidID].z;
                    //float localSpeedSqr = localVelocity.x * localVelocity.x + localVelocity.y * localVelocity.y + localVelocity.z * localVelocity.z;

                    ////! If it is too close
                    //if (dsq <= _SystemOptions.SeparationRadius)
                    //{
                    //    if (speedSqr >= localSpeedSqr)
                    //        newVelocity -= boidDir * t;

                    //}
                    //else if (dsq > _SystemOptions.CohesionRadius)    //! If it is too far
                    //{
                    //    if (speedSqr <= localSpeedSqr)
                    //        newVelocity += boidDir * _SystemOptions.ChangeRate * t;
                    //}

                    ClampMagnitude(ref newVelocity, _SystemOptions.VelocityLimits.x, _SystemOptions.VelocityLimits.y);

                    _Vel[boidID] = newVelocity;
                } while (iterator.MoveNext());
            }
        }

        (int index, float val) minValue(ref float3 vec)
        {
            float3 temp = vec * vec;
            int index = temp.x < temp.y ? (temp.x < temp.z ? 0 : 2) : (temp.y < temp.z ? 1 : 2);
            return (index, math.abs(vec[index]));
        }

        public static void ClampMagnitude(ref float3 v, float max, float min)
        {
            double sm = v.x * v.x + v.y * v.y + v.z * v.z;

            v = sm > (double)max * (double)max ? math.normalize(v) * max : math.normalize(v) * min;
        }
    }
    
}