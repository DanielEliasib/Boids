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
        [ReadOnly] public NativeArray<float3> _Dir;
        [ReadOnly] public NativeArray<float> _Vel;

        [ReadOnly] public float deltaTime;
        [ReadOnly] public float3 _AreaSize;

        public void Execute(int index)
        {
            _Pos[index] += _Dir[index] * _Vel[index] * deltaTime;

            //float3 change = float3.zero;

            //if (math.abs(_Pos[index].x) > _AreaSize.x * 0.5f)
            //{
            //    var sign = math.sign(_Pos[index].x);
            //    change.x -= sign * _AreaSize.x;
            //}

            //if (math.abs(_Pos[index].y) > _AreaSize.y * 0.5f)
            //{
            //    var sign = math.sign(_Pos[index].y);
            //    change.y -= sign * _AreaSize.y;
            //}

            //if (math.abs(_Pos[index].z) > _AreaSize.z * 0.5f)
            //{
            //    var sign = math.sign(_Pos[index].z);
            //    change.z -= sign * _AreaSize.z;
            //}

            //_Pos[index] += change;
        }
    }
}


