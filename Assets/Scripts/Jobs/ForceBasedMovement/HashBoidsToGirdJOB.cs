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
        public NativeMultiHashMap<int, int>.ParallelWriter _GridToBoidMap;
        public NativeArray<int> _BoidToGridMap;

        [ReadOnly] public NativeArray<float3> _OldPos;
        [ReadOnly] public float3 _CubeSize;
        [ReadOnly] public int3 _Divitions;
        [ReadOnly] public float3 _AreaSize;

        public void Execute(int index)
        {
            var absPos = math.abs(_OldPos[index]);

            if (absPos.x < _AreaSize.x*0.5f  && absPos.y < _AreaSize.y * 0.5f && absPos.z < _AreaSize.z * 0.5f)
            {
                float3 relativePosition = _AreaSize * 0.5f + _OldPos[index];
                int3 key = new int3(
                    (int)(relativePosition.x / _CubeSize.x),
                    (int)(relativePosition.y / _CubeSize.y),
                    (int)(relativePosition.z / _CubeSize.z));

                int flatKey = key.z + key.y * _Divitions.z + key.x * _Divitions.z * _Divitions.y;

                _BoidToGridMap[index] = flatKey;

                _GridToBoidMap.Add(flatKey, index);
            }
        }

    }
}