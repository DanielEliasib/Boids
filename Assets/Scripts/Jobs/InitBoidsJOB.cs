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
        public NativeArray<float3> _Dir;
        public NativeArray<float> _Vel;

        [ReadOnly] public Unity.Mathematics.Random _Rand;

        [ReadOnly] public float _Rad;
        [ReadOnly] public float2 _VelLimit;

        public void Execute(int index)
        {
            _Pos[index] = _Rand.NextFloat3Direction() * _Rand.NextFloat(0.0f, 1.0f) * _Rad;
            _Dir[index] = _Rand.NextFloat3Direction();
            _Vel[index] = _Rand.NextFloat(_VelLimit.x, _VelLimit.y);
        }
    }
}