using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    public struct GenerateGridJOB : IJobParallelFor
    {
        public NativeArray<float3> _CubeCenters;
        [ReadOnly] public float3 _CubeSize;
        [ReadOnly] public int3 _Divitions;
        [ReadOnly] public float3 _InitialCubeCenter;

        public void Execute(int index)
        {
            int k = index % _Divitions.z;
            int j = (index / _Divitions.z) % _Divitions.y;
            int i = index / (_Divitions.y * _Divitions.z);

            _CubeCenters[index] = _InitialCubeCenter + new float3(_CubeSize.x * i, _CubeSize.y * j, _CubeSize.z * k);
        }
    }
}