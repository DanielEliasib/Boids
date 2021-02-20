using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AL.BoidSystem.Jobs
{
    public struct UpdateBoidsJOB : IJobParallelFor
    {
        public NativeArray<float3> _Pos;
        [ReadOnly] public NativeArray<float3> _Vel;

        [ReadOnly] public float deltaTime;

        public void Execute(int index)
        {
            _Pos[index] += _Vel[index] * deltaTime;
            //_Pos[index] = math.lerp(_Pos[index], _Pos[index] + _Vel[index] * deltaTime, 25 * deltaTime);
        }
    }
}


