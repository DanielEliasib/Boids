using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    public struct GenerateRayCastCommandsJOB : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> _RayDirections;
        [ReadOnly] public NativeArray<float3> _Pos;
        [ReadOnly] public NativeArray<Matrix4x4> _TransformMatrices;

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

            float3 pos = _Pos[boidIndex];
            float3 dir = _TransformMatrices[boidIndex].rotation * _RayDirections[rayIndex];

            _RayCastCommands[index] = new RaycastCommand(pos, dir, _VisDistance, _HitMask, 1);
        }
    }
}