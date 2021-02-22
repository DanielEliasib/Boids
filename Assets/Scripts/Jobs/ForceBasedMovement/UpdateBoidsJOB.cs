using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

using Unity.Burst;

namespace AL.BoidSystem.Jobs
{
    public struct UpdateBoidsJOB : IJobParallelFor
    {
        public NativeArray<float3> _Position;
        public NativeArray<float3> _Velocity;
        public NativeArray<float4x4> _TransMatrix;

        [ReadOnly] public NativeArray<float3> _OldPosition;
        [ReadOnly] public NativeArray<float3> _OldVelocity;

        [ReadOnly] public NativeArray<float3> _CorrectionForce;
        [ReadOnly] public float deltaTime;
        [ReadOnly] public BoidSystemOptions _SystemOptions;

        [ReadOnly] private static float3 _One = new float3(1, 1, 1);
        [ReadOnly] private static float3 _Up = new float3(0, 1, 0);

        public void Execute(int boidID)
        {
            _Position[boidID] = _OldPosition[boidID] + _OldVelocity[boidID] * deltaTime + _CorrectionForce[boidID] * deltaTime * deltaTime * 0.5f;
            float3 tempVel = _OldVelocity[boidID] + _CorrectionForce[boidID] * deltaTime;
            float3 normalized = ClampMagnitude(ref tempVel, _SystemOptions.VelocityLimits.x, _SystemOptions.VelocityLimits.y);
            _Velocity[boidID] = tempVel;
            _TransMatrix[boidID] = float4x4.TRS(_Position[boidID], quaternion.LookRotation(normalized, _Up), _One);
        }

        public static float3 ClampMagnitude(ref float3 v, float max, float min)
        {
            double sm = v.x * v.x + v.y * v.y + v.z * v.z;
            float3 normalized = math.normalize(v);

            v = sm > (double)max * (double)max ? normalized * max : normalized * min;
            return normalized;
        }
    }
}