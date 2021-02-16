using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace AL.BoidSystem
{
    public struct SimulationArea
    {
        public int3 Divitions {get { return _Divs; } }
        public float3 Center {get { return _Center; } }
        public float3 Size { get { return _Size; } }
        public float3 InnerCubeSize { get { return new float3(_Size.x / Divitions.x, _Size.y / Divitions.y, _Size.z / Divitions.z); } }
        public int NumberOfCubes { get { return _NumberCubes; } }

        int3 _Divs;
        float3 _Center;
        float3 _Size;
        int _NumberCubes;

        public SimulationArea(int3 numberOfDivitions, float3 center, float3 size)
        {
            if (numberOfDivitions.x <= 0 || numberOfDivitions.y <= 0 || numberOfDivitions.z <= 0)
                throw new System.Exception($"Number of divitions must be positive. Divs: {numberOfDivitions}");

            if (size.x <= 0 || size.y <= 0 || size.z <= 0)
                throw new System.Exception($"Size must be positive. Size: {size}");

            if (float.IsNaN(center.x) || float.IsNaN(center.y) || float.IsNaN(center.z))
                throw new System.Exception($"Center must be a number. Center: {center}");

            _Divs = numberOfDivitions;
            _Center = center;
            _Size = size;

            _NumberCubes = numberOfDivitions.x * numberOfDivitions.y * numberOfDivitions.z;
        }
    }
}

