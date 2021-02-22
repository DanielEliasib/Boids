using System.Collections;
using System.Collections.Generic;

using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AL.BoidSystem.Jobs
{
    public struct InitBoidsJOB : IJobParallelFor
    {
        public NativeArray<float3> _Pos;
        public NativeArray<float3> _Vel;
        public NativeArray<float4x4> _Mat;

        [ReadOnly] public Random _Rand;

        [ReadOnly] public float _Rad;
        [ReadOnly] public float2 _VelLimit;

        private static float3 _One = new float3(1, 1, 1);
        private static float3 _Up = new float3(0, 1, 0);

        public void Execute(int index)
        {
            var rand = new Random((uint)((index+1)*_Rand.NextInt()));

            _Pos[index] = rand.NextFloat3Direction() * rand.NextFloat(0.0f, 1.0f) * _Rad;
            _Vel[index] = rand.NextFloat3Direction() * rand.NextFloat(_VelLimit.x, _VelLimit.y)*5;
            _Mat[index] = float4x4.TRS(_Pos[index], quaternion.LookRotation(_Vel[index], _Up), _One);
        }
    }
}