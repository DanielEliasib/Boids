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
        public float SeparationRadius { get { return _SepRad; } set { _SepRad = value; _SepRadSqr = value * value; } }
        public float SeparationRadiusSqr { get { return _SepRadSqr; } private set {; } }
        public float CohesionRadius { get; set; }
        public float NoiseMagnitude { get; set; }

        private float _SepRadSqr;
        private float _SepRad;
    }
}