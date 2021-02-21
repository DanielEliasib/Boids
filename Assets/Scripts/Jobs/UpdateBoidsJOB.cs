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
        public NativeArray<float3> _Vel;
        public NativeArray<Matrix4x4> _Mat;

        [ReadOnly] public NativeArray<float3> _OldVel;
        [ReadOnly] public float deltaTime;
        [ReadOnly] public float3 _One;

        [ReadOnly] public BoidSystemOptions _SystemOptions;

        public void Execute(int index)
        {
            float3 vel = math.lerp(_OldVel[index], _Vel[index], deltaTime * _SystemOptions.ChangeRate);
            ClampMagnitude(ref vel, _SystemOptions.VelocityLimits.x, _SystemOptions.VelocityLimits.y);
            _Vel[index] = vel;

            //! Interpolate
            _Pos[index] += _Vel[index] * deltaTime;

            _Mat[index] = Matrix4x4.TRS(_Pos[index], Quaternion.LookRotation(_Vel[index]), Vector3.one);
            //_Pos[index] = math.lerp(_Pos[index], _Pos[index] + _Vel[index] * deltaTime, 25 * deltaTime);
        }

        public static void ClampMagnitude(ref float3 v, float max, float min)
        {
            double sm = v.x * v.x + v.y * v.y + v.z * v.z;

            v = sm > (double)max * (double)max ? math.normalize(v) * max : math.normalize(v) * min;
        }
    }
}


