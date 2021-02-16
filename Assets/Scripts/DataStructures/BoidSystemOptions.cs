using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Mathematics;

namespace AL.BoidSystem
{
    public struct BoidSystemOptions
    {
        public float2 VelocityLimits { get; set; }
        public float ObstacleVision { get; set; }
        public float ChangeRate { get; set; }
        public float SeparationRadius { get; set; }
        public float CohesionRadius { get; set; }

    }
}