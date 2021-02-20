using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AL.BoidSystem.Jobs
{
    public struct InitBoidsJOB : IJobParallelFor
    {
        public NativeArray<float3> _Pos;
        public NativeArray<float3> _Vel;

        [ReadOnly] public Unity.Mathematics.Random _Rand;

        [ReadOnly] public float _Rad;
        [ReadOnly] public float2 _VelLimit;

        public void Execute(int index)
        {
            var rand = new Unity.Mathematics.Random((uint)((index+1)*_Rand.NextInt()));

            _Pos[index] = rand.NextFloat3Direction() * rand.NextFloat(0.0f, 1.0f) * _Rad;
            _Vel[index] = rand.NextFloat3Direction() * rand.NextFloat(_VelLimit.x, _VelLimit.y);
        }
    }
}