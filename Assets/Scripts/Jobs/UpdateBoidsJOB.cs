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
        public NativeArray<Matrix4x4> _Mat;
        [ReadOnly] public NativeArray<float3> _Vel;

        [ReadOnly] public float deltaTime;
        [ReadOnly] public float3 _One;

        public void Execute(int index)
        {
            _Pos[index] += _Vel[index] * deltaTime;
            _Mat[index] = Matrix4x4.TRS(_Pos[index], Quaternion.LookRotation(_Vel[index]), Vector3.one);
            //_Pos[index] = math.lerp(_Pos[index], _Pos[index] + _Vel[index] * deltaTime, 25 * deltaTime);
        }
    }
}


