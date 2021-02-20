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
        //! Main Boid Data Holders
        private NativeArray<float3> _Positions;
        private NativeArray<float3> _Velocities;

        private int _NOfBodies;

        private BoidSystemOptions _SystemOptions;

        //! Simulation Grid Data
        private SimulationArea _SimArea;

        private NativeArray<float3> _SimCubeCenters;
        private NativeMultiHashMap<int, int> _GridToBoidsMap;

        //! Global JOBS
        private UpdateBoidsJOB _UpdateJOB;
        private FlockUpdateJOB _FlockJOB;

        //! Global JOB Handles
        JobHandle updateHandle;

        //! Baking Data
        private ComputeShader _DataCopyShader;
        private ComputeBuffer _PointBuffer;
        private ComputeBuffer _DirecBuffer;
        private int _KernelIndex;
        private string _KernelName = "CopyKernel";

        private RenderTexture _InternalPointTexture;
        private RenderTexture _InternalDirecTexture;

        private bool _CopyInitialize;

        public BoidSystem(int nOfBoids, SimulationArea simArea, ref BoidSystemOptions systemOptions)
        {
            _CopyInitialize = false;

            _SimArea = simArea;

            _SystemOptions = systemOptions;

            _NOfBodies = nOfBoids;
            Initialize(_NOfBodies, 445);
        }

        public void Dispose()
        {
            _Positions.Dispose();
            _Velocities.Dispose();
            _SimCubeCenters.Dispose();

            try
            {
                _GridToBoidsMap.Dispose();
            }
            catch { }
        }

        private void InitializeSimulationGrid()
        {
            _GridToBoidsMap = new NativeMultiHashMap<int, int>(_SimArea.NumberOfCubes, Allocator.TempJob);
            _SimCubeCenters = new NativeArray<float3>(_SimArea.NumberOfCubes, Allocator.Persistent);

            GenerateGridJOB _GenGridJOB = new GenerateGridJOB()
            {
                _CubeCenters = _SimCubeCenters,
                _CubeSize = _SimArea.InnerCubeSize,
                _Divitions = _SimArea.Divitions,
                _InitialCubeCenter = _SimArea.Center - _SimArea.Size * 0.5f + _SimArea.InnerCubeSize * 0.5f
            };

            _GenGridJOB.Schedule(_SimArea.NumberOfCubes, 8).Complete();
        }

        private void Initialize(int n, uint randSeed)
        {
            InitializeSimulationGrid();

            _Positions = new NativeArray<float3>(n, Allocator.Persistent);
            _Velocities = new NativeArray<float3>(n, Allocator.Persistent);

            InitBoidsJOB _InitJob = new InitBoidsJOB()
            {
                _Pos = _Positions,
                _Vel = _Velocities,
                _Rand = new Unity.Mathematics.Random(randSeed),
                _Rad = math.min(math.min(_SimArea.Size.x, _SimArea.Size.y), _SimArea.Size.z)*0.45f,
                _VelLimit = _SystemOptions.VelocityLimits
            };

            _UpdateJOB = new UpdateBoidsJOB()
            {
                _Pos = _Positions,
                _Vel = _Velocities,
                deltaTime = 0
            };

            _FlockJOB = new FlockUpdateJOB()
            {
                _Vel = _Velocities,
                _SystemOptions = _SystemOptions,
                _SimArea = _SimArea,
                rand = new Unity.Mathematics.Random(randSeed*randSeed)
            };

            _InitJob.Schedule(n, 8).Complete();
        }

        public void UpdateSystem()
        {
            // Debug
            var watch = new System.Diagnostics.Stopwatch();
            watch.Reset();
            watch.Start();

            float deltaTime = Time.fixedDeltaTime;

            //! Obstacle Findidng
            NativeArray<RaycastHit> _ObstacleHits = new NativeArray<RaycastHit>(_NOfBodies, Allocator.TempJob);
            NativeArray<RaycastCommand> _CastCommands = new NativeArray<RaycastCommand>(_NOfBodies, Allocator.TempJob);

            GenerateRayCastCommandsJOB _GenerateCastJob = new GenerateRayCastCommandsJOB() {
                _Pos = _Positions,
                _Vel = _Velocities,
                _HitMask = LayerMask.NameToLayer("BoidObs"),
                _RayCastCommands = _CastCommands,
                _VisDistance = _SystemOptions.ObstacleVision
            };

            // Schedule both jobs
            JobHandle genCastHandle = _GenerateCastJob.Schedule(_NOfBodies, 8);
            JobHandle rayCastHandle = RaycastCommand.ScheduleBatch(_CastCommands, _ObstacleHits, 8, genCastHandle);

            //! Boid grid hashing
            _GridToBoidsMap.Dispose();
            _GridToBoidsMap = new NativeMultiHashMap<int, int>(_NOfBodies, Allocator.TempJob);

            HashBoidsToGirdJOB _HashBoidsJOB = new HashBoidsToGirdJOB()
            {
                _CubeSize = _SimArea.InnerCubeSize,
                _Divitions = _SimArea.Divitions,
                _Pos = _Positions,
                _CubeToBoidMap = _GridToBoidsMap.AsParallelWriter(),
                _AreaSize = _SimArea.Size
            };

            // Scheduling
            JobHandle hashBoidsHandle = _HashBoidsJOB.Schedule(_Positions.Length, 8, rayCastHandle);

            //! Flock behaviour job
            NativeArray<float3> oldPos = new NativeArray<float3>(_Positions, Allocator.TempJob);
            NativeArray<float3> oldVel = new NativeArray<float3>(_Velocities, Allocator.TempJob);

            _FlockJOB._OldPos = oldPos;
            _FlockJOB._OldVel = oldVel;
            _FlockJOB._GridToBoidsMap = _GridToBoidsMap;
            _FlockJOB._RayCastHits = _ObstacleHits;
            _FlockJOB.deltaTime = deltaTime;
            _FlockJOB._SystemOptions = _SystemOptions;
            _FlockJOB.rand = new Unity.Mathematics.Random((uint)UnityEngine.Random.Range(1,15478));
            _FlockJOB._VisDistance = _SystemOptions.ObstacleVision;

            // Scheduling
            JobHandle updateFlockHandle = _FlockJOB.Schedule(_SimArea.NumberOfCubes, 8, hashBoidsHandle);

            //! Simple update. If the above calculations take more time, they might be schedule every n frames, but this can still run every frame.
            _UpdateJOB.deltaTime = deltaTime;

            // Scheduling
            updateHandle = _UpdateJOB.Schedule(_NOfBodies, 8, updateFlockHandle);

            //! Complete All Jobs
            genCastHandle.Complete();
            rayCastHandle.Complete();
            hashBoidsHandle.Complete();
            updateFlockHandle.Complete();
            updateHandle.Complete();

            //! Dispose temporary collections. This might be improved and may not need to be disposed every time, maybe some swapping with the current positions
            oldPos.Dispose();
            oldVel.Dispose();
            
            _CastCommands.Dispose();
            _ObstacleHits.Dispose();

            // Debug
            watch.Stop();

            Debug.Log($"TIme ellapsed: {watch.Elapsed.TotalMilliseconds}");
        }

        public void InitializeCopyShader(ref RenderTexture positionMap, ref RenderTexture directionMap)
        {
            _InternalPointTexture = positionMap;
            _InternalDirecTexture = directionMap;

            _DataCopyShader = (ComputeShader)Resources.Load("CopyArrayToTexture");

            Debug.Log(_DataCopyShader);

            _KernelIndex = _DataCopyShader.FindKernel(_KernelName);

            _DataCopyShader.SetTexture(_KernelIndex, "_PointResult", _InternalPointTexture);
            _DataCopyShader.SetTexture(_KernelIndex, "_DirecResult", _InternalDirecTexture);

            _DataCopyShader.SetInt("width", _InternalPointTexture.width);

            _DataCopyShader.SetFloat("maxValue", half.MaxValue);

            _PointBuffer = new ComputeBuffer(1, 3 * sizeof(float));
            _DirecBuffer = new ComputeBuffer(1, 3 * sizeof(float));

            _CopyInitialize = true;
        }

        public void BakeDataToRenderTexture()
        {
            if (_CopyInitialize)
            {
                _PointBuffer.Dispose();
                _PointBuffer = new ComputeBuffer(_Positions.Length, 3 * sizeof(float));
                _DataCopyShader.SetBuffer(_KernelIndex, "_PointData", _PointBuffer);

                _DirecBuffer.Dispose();
                _DirecBuffer = new ComputeBuffer(_Velocities.Length, 3 * sizeof(float));
                _DataCopyShader.SetBuffer(_KernelIndex, "_DirecData", _DirecBuffer);

                _DataCopyShader.SetInt("arrayLenght", _Positions.Length);

                _PointBuffer.SetData(_Positions.ToArray());

                _DataCopyShader.Dispatch(_KernelIndex, _InternalPointTexture.width / 8, _InternalPointTexture.height, 1);
            }
            else
            {
                throw new System.Exception("Copy shader was not initialized correctly.");
            }
        }

        public void DrawSystem()
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(_Positions[0], _Positions[0] + _Velocities[0]);

            Gizmos.color = Color.white;
            Gizmos.DrawWireSphere(_Positions[0], _SystemOptions.ObstacleVision);

            //Gizmos.color = Color.red;
            //Gizmos.DrawWireSphere(_Positions[0], _SystemOptions.CohesionRadius);

            //Gizmos.color = Color.magenta;
            //Gizmos.DrawWireSphere(_Positions[0], _SystemOptions.SeparationRadius);


            //for (int i = 0; i < _NOfBodies; i++)
            //{
            //    Gizmos.color = Color.white;
            //    Gizmos.DrawWireSphere(_Positions[i], 0.05f);
            //    Gizmos.color = i == 0 ? Color.cyan : Color.red;
            //    Gizmos.DrawLine(_Positions[i], _Positions[i] + _Directions[i] * _Velocities[i] * 0.25f);
            //}
        }

        public void DrawSimulationArea(bool drawUsedCubes)
        {
            if (drawUsedCubes)
            {
                float3 cubeSize = _SimArea.InnerCubeSize;
                

                for (int i = 0; i < _SimCubeCenters.Length; i++)
                {
                    if (_GridToBoidsMap.IsCreated && _GridToBoidsMap.ContainsKey(i))
                    {
                        Gizmos.color = Color.cyan;
                        Gizmos.DrawWireCube(_SimCubeCenters[i], cubeSize);

                        Gizmos.color = Color.red;
                        Gizmos.DrawWireSphere(_SimCubeCenters[i], _SystemOptions.CohesionRadius);

                        Gizmos.color = Color.magenta;
                        Gizmos.DrawWireSphere(_SimCubeCenters[i], _SystemOptions.SeparationRadius);
                    }
                    
                }
            }

            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(_SimArea.Center, _SimArea.Size);
        }

        public void UpdateSystemOptions(ref BoidSystemOptions systemOptions)
        {
            _SystemOptions = systemOptions;
        }
    }
}
