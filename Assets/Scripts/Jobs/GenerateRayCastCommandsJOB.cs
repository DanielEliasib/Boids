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
        [ReadOnly] public NativeArray<float3> _Pos;
        [ReadOnly] public NativeArray<float3> _Vel;

        [ReadOnly] public float _VisDistance;
        [ReadOnly] public int _HitMask;

        public NativeArray<RaycastCommand> _RayCastCommands;

        public void Execute(int index)
        {
            _RayCastCommands[index] = new RaycastCommand(_Pos[index], _Vel[index], _VisDistance, _HitMask, 1);
        }
    }
}