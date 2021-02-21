using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

public struct BoidCollisionJOB : IJobParallelFor
{
    public NativeArray<float3> _NewVel;

    [ReadOnly] public NativeArray<RaycastHit> _RayCastHits;
    [ReadOnly] public NativeArray<float3> _RayDirections;
    [ReadOnly] public NativeArray<Matrix4x4> _TransformMatrices;

    [ReadOnly] public int _NumberOfBoids;
    [ReadOnly] public int _NumberOfRays;

    //: Index goes through every boid
    //! y * width + x
    public void Execute(int index)
    {
        _NewVel[index] = float3.zero;

        //! Check the first one
        int realIndex = 0 * _NumberOfBoids + index;
        float3 normal = _RayCastHits[realIndex].normal;
        float3 newDir = float3.zero;

        bool freeFoud = false;
        bool collision = false;
        if (math.abs(normal.x + normal.y  + normal.z) > 4*float.Epsilon)
        {
            collision = true;

            for (int i = 1; i < _NumberOfRays; i++)
            {
                realIndex+= _NumberOfBoids;
                normal = _RayCastHits[realIndex].normal;
                if (math.abs(normal.x + normal.y + normal.z) <= 4 * float.Epsilon)
                {
                    newDir += (float3)(_TransformMatrices[index].rotation * _RayDirections[i]);
                    freeFoud = true;
                    break;
                }
            }
        }

        if (freeFoud)
            _NewVel[index] += newDir*3;
        else if (collision)
            _NewVel[index] += normal;
            
    }
}
