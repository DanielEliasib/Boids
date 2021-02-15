using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

namespace AL.BoidSystem.Jobs
{
    public struct HashBoidsToGirdJOB : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> _Pos;
        public NativeMultiHashMap<int, int>.ParallelWriter _CubeToBoidMap;
        public float3 _CubeSize;
        public int3 _Divitions;
        public float3 _AreaSize;

        public void Execute(int index)
        {
            float3 relativePosition = _AreaSize * 0.5f + _Pos[index];
            int3 key = new int3(
                (int)(relativePosition.x / _CubeSize.x), 
                (int)(relativePosition.y / _CubeSize.y),
                (int)(relativePosition.z / _CubeSize.z));

            _CubeToBoidMap.Add(key.z + key.y * _Divitions.z + key.x * _Divitions.z * _Divitions.y, index);
        }

    }
}