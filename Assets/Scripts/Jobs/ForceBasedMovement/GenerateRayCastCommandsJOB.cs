using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

using Unity.Burst;

namespace AL.BoidSystem.Jobs
{
    //: Maybe will make it so that it RayCast more only if the first one finds something.
    [BurstCompile]
    public struct GenerateRayCastCommandsJOB : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> _RayDirections;
        [ReadOnly] public NativeArray<float3> _OldPos;
        [ReadOnly] public NativeArray<float4x4> _TransMatrices;

        [ReadOnly] public float _VisDistance;
        [ReadOnly] public int _HitMask;
        [ReadOnly] public int _NumberOfBoids;

        public NativeArray<RaycastCommand> _RayCastCommands;

        //: Consider as a 2D array with width number of boids and height number of rays

        //! y = index / width;  rayIndex
        //! x = index % width;  boidIndex

        public void Execute(int index)
        {
            int rayIndex = index / _NumberOfBoids;
            int boidIndex = index % _NumberOfBoids;

            float3 pos = _OldPos[boidIndex];
            float3 dir = math.mul(_TransMatrices[boidIndex], new float4(_RayDirections[rayIndex], 0)).xyz;

            _RayCastCommands[index] = new RaycastCommand(pos, dir, _VisDistance, _HitMask, 1);
        }

    }
}