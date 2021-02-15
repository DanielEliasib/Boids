using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections;

using AL.BoidSystem.Jobs;

namespace AL.BoidSystem
{
    public unsafe class BoidSystem
    {
        NativeArray<float3> _Positions;
        NativeArray<float3> _Directions;
        NativeArray<float> _Velocities;

        NativeArray<float3> _SimCubesCenter;
        NativeMultiHashMap<int, int> _GridToBoidsMap;

        private int _NOfBodies;
        private SimulationArea _SimArea;

        private float _SearchRad;

        private UpdateBoidsJOB _UpdateJOB;
        private FlockUpdateJOB _FlockJOB;

        JobHandle _UpdateHandle;

        public BoidSystem(int nOfBoids, SimulationArea simArea)
        {
            _SimArea = simArea;

            _NOfBodies = nOfBoids;
            Initialize(_NOfBodies);
        }

        public void Dispose()
        {
            _Positions.Dispose();
            _Directions.Dispose();
            _Velocities.Dispose();
            _SimCubesCenter.Dispose();

            try
            {
                _GridToBoidsMap.Dispose();
            }
            catch { }
        }

        private void Initialize(int n)
        {
            _GridToBoidsMap = new NativeMultiHashMap<int, int>(_SimArea.NumberOfCubes, Allocator.TempJob);

            _SimCubesCenter = new NativeArray<float3>(_SimArea.NumberOfCubes, Allocator.Persistent);

            GenerateGridJOB _GenGridJOB = new GenerateGridJOB()
            {
                _CubeCenters = _SimCubesCenter,
                _CubeSize = _SimArea.InnerCubeSize,
                _Divitions = _SimArea.Divitions,
                _InitialCubeCenter = _SimArea.Center - _SimArea.Size * 0.5f + _SimArea.InnerCubeSize * 0.5f
            };

            _Positions = new NativeArray<float3>(n, Allocator.Persistent);
            _Directions = new NativeArray<float3>(n, Allocator.Persistent);
            _Velocities = new NativeArray<float>(n, Allocator.Persistent);

            InitBoidsJOB _InitJob = new InitBoidsJOB()
            {
                _Pos = _Positions,
                _Dir = _Directions,
                _Vel = _Velocities,
                _Rand = new Unity.Mathematics.Random(452),
                _Rad = 2
            };

            _UpdateJOB = new UpdateBoidsJOB()
            {
                _Pos = _Positions,
                _Dir = _Directions,
                _Vel = _Velocities,
                _AreaSize = _SimArea.Size,
                deltaTime = 0
            };

            _FlockJOB = new FlockUpdateJOB()
            {
                _Dir = _Directions,
                _Vel = _Velocities,
                minVel = 0.1f,
                maxVel = 3.0f,
                rand = new Unity.Mathematics.Random(10)
            };

            _InitJob.Schedule(n, 8).Complete();
            _GenGridJOB.Schedule(_SimArea.NumberOfCubes, 8).Complete();
        }

        public void UpdateSystem()
        {
            try
            {
                _GridToBoidsMap.Dispose();
            }catch{Debug.Log("Oopss!");}

            _GridToBoidsMap = new NativeMultiHashMap<int, int>(_NOfBodies, Allocator.TempJob);

            HashBoidsToGirdJOB _HashBoidsJOB = new HashBoidsToGirdJOB()
            {
                _CubeSize = _SimArea.InnerCubeSize,
                _Divitions = _SimArea.Divitions,
                _Pos = _Positions,
                _CubeToBoidMap = _GridToBoidsMap.AsParallelWriter(),
                _AreaSize = _SimArea.Size
            };

            _HashBoidsJOB.Schedule(_Positions.Length, 8).Complete();

            NativeArray<float3> oldPos = new NativeArray<float3>(_Positions, Allocator.TempJob);
            NativeArray<float3> oldDir = new NativeArray<float3>(_Directions, Allocator.TempJob);
            NativeArray<float> oldVel = new NativeArray<float>(_Velocities, Allocator.TempJob);

            _FlockJOB._OldPos = oldPos;
            _FlockJOB._OldDir = oldDir;
            _FlockJOB._OldVel = oldVel;
            _FlockJOB._GridToBoidsMap = _GridToBoidsMap;
            _FlockJOB.deltaTime = Time.deltaTime;
            _FlockJOB.changeVelocity = 0.05f;
            _FlockJOB.separationRad = 0.15f;
            _FlockJOB.cohesionRad = 0.5f;

            JobHandle _UpdateFlockHandle = _FlockJOB.Schedule(_SimArea.NumberOfCubes, 8);

            _UpdateJOB.deltaTime = Time.deltaTime;

            _UpdateHandle = _UpdateJOB.Schedule(_NOfBodies, 8, _UpdateFlockHandle);

            _UpdateFlockHandle.Complete();
            _UpdateHandle.Complete();

            oldPos.Dispose();
            oldDir.Dispose();
            oldVel.Dispose();
        }

        public void DrawSystem()
        {
            for (int i = 0; i < _NOfBodies; i++)
            {
                Gizmos.color = Color.white;
                Gizmos.DrawWireSphere(_Positions[i], 0.05f);
                Gizmos.color = i == 0? Color.cyan : Color.red;
                Gizmos.DrawLine(_Positions[i], _Positions[i] + _Directions[i] * _Velocities[i]*0.25f);
            }
        }

        public void DrawSimulationArea()
        {
            //float3 cubeSize = _SimArea.InnerCubeSize;
            //Gizmos.color = Color.cyan;

            //for (int i = 0; i < _SimCubesCenter.Length; i++)
            //{
            //    if(_GridToBoidsMap.IsCreated && _GridToBoidsMap.ContainsKey(i))
            //        Gizmos.DrawWireCube(_SimCubesCenter[i], cubeSize);
            //}
                

            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(_SimArea.Center, _SimArea.Size);
        }
    }
}
