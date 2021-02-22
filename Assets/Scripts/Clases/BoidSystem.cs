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

        private NativeArray<float3> _OldPositions;
        private NativeArray<float3> _OldVelocities;

        private NativeArray<float3> _CorrectionForces;
        private NativeArray<float4x4> _Matrices;

        NativeArray<RaycastCommand> _RayCastCommands;
        NativeArray<RaycastHit> _ObstacleHits;

        private NativeArray<float3> _RayDirections;
        private int _NRays = 8*2;

        private int _NBoids;

        private BoidSystemOptions _SystemOptions;

        //! Simulation Grid Data
        private SimulationArea _SimArea;
        private List<GameObject> _SimulationAreaWalls;
        private GameObject _QuadWallPrefab;

        private NativeArray<float3> _SimCubeCenters;
        private NativeMultiHashMap<int, int> _GridToBoidsMap;
        private NativeArray<int> _BoidToGridMap;

        //! Global JOBS
        private GenerateRayCastCommandsJOB _GenerateRaycastJOB;
        private CollisionForceJOB _CollisionJOB;
        private BoidAlignJOB _AlignJOB;
        private UpdateBoidsJOB _UpdateJOB;

        //! Global JOB Handles
        JobHandle updateHandle;

        private float3[] _ZeroArray;

        public BoidSystem(int nOfBoids, SimulationArea simArea, ref BoidSystemOptions systemOptions)
        {
            _SimArea = simArea;

            _SystemOptions = systemOptions;

            _NBoids = nOfBoids;
            Initialize(_NBoids, 1587);
        }

        public void Dispose()
        {
            _Positions.Dispose();
            _OldPositions.Dispose();
            _Velocities.Dispose();
            _OldVelocities.Dispose();
            _CorrectionForces.Dispose();
            _Matrices.Dispose();

            _RayCastCommands.Dispose();
            _SimCubeCenters.Dispose();
            _RayDirections.Dispose();
            _ObstacleHits.Dispose();

            try
            {
                _GridToBoidsMap.Dispose();
                _BoidToGridMap.Dispose();
            }
            catch { }
        }

        private void InitializeSimulationGrid()
        {
            _GridToBoidsMap = new NativeMultiHashMap<int, int>(_SimArea.NumberOfCubes*_NBoids, Allocator.Persistent);
            _BoidToGridMap = new NativeArray<int>(_NBoids, Allocator.Persistent);
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
            _ZeroArray = new float3[n];

            //! Initialize Simulation Grid
            InitializeSimulationGrid();
            GenerateSimulationWalls();

            //! Initialize data collections
            _Positions = new NativeArray<float3>(n, Allocator.Persistent);
            _Velocities = new NativeArray<float3>(n, Allocator.Persistent);
            _OldPositions = new NativeArray<float3>(n, Allocator.Persistent);
            _OldVelocities = new NativeArray<float3>(n, Allocator.Persistent);

            _CorrectionForces = new NativeArray<float3>(n, Allocator.Persistent);
            _Matrices = new NativeArray<float4x4>(n, Allocator.Persistent);
            _RayDirections = new NativeArray<float3>(_NRays, Allocator.Persistent);
            _RayCastCommands = new NativeArray<RaycastCommand>(_NBoids * _NRays, Allocator.Persistent);
            _ObstacleHits = new NativeArray<RaycastHit>(_NBoids * _NRays, Allocator.Persistent);

            InitBoidsJOB _InitJob = new InitBoidsJOB()
            {
                _Pos = _Positions,
                _Vel = _Velocities,
                _Mat = _Matrices,
                _Rand = new Unity.Mathematics.Random(randSeed),
                _Rad = math.min(math.min(_SimArea.Size.x, _SimArea.Size.y), _SimArea.Size.z)*0.45f,
                _VelLimit = _SystemOptions.VelocityLimits
            };

            GenerateRayDirectionsJOB _DirectionsJOB = new GenerateRayDirectionsJOB()
            {
                _TotalPoints = _NRays * 2,
                golden2 = (1.0f + Mathf.Sqrt(5)),
                _RayDirections = _RayDirections
            };

            _GenerateRaycastJOB = new GenerateRayCastCommandsJOB()
            {
                _OldPos = _OldPositions,
                _HitMask = LayerMask.NameToLayer("BoidObs"),
                _RayCastCommands = _RayCastCommands,
                _VisDistance = _SystemOptions.ObstacleVision,
                _NumberOfBoids = _NBoids,
                _RayDirections = _RayDirections,
                _TransMatrices = _Matrices
            };

            _CollisionJOB = new CollisionForceJOB()
            {
                _CorrectionForce = _CorrectionForces,
                _RayCastHits = _ObstacleHits,
                _RayDirections = _RayDirections,
                _TransMatrices = _Matrices,
                _OldVelocity = _OldVelocities,
                _NumberOfBoids = _NBoids,
                _NumberOfRays = _NRays
            };

            _AlignJOB = new BoidAlignJOB()
            {
                _BoidToGridMap = _BoidToGridMap,
                _CorrectionForce = _CorrectionForces,
                _GridToBoidsMap = _GridToBoidsMap,
                _OldPosition = _OldPositions,
                _OldVelocity = _OldVelocities,
                _SimArea = _SimArea
            };

            _UpdateJOB = new UpdateBoidsJOB()
            {
                _CorrectionForce = _CorrectionForces,
                _TransMatrix = _Matrices,
                _Position = _Positions,
                _Velocity = _Velocities,
                deltaTime = 0
            };

            _InitJob.Schedule(n, 8).Complete();
            _DirectionsJOB.Schedule(_NRays, 8).Complete();
        }

        public void UpdateSystem()
        {
            //! Swapping
            Swap(ref _OldPositions, ref _Positions);
            Swap(ref _OldVelocities, ref _Velocities);

            //! Obstacle handling
            _GenerateRaycastJOB._OldPos = _OldPositions;
            JobHandle genCommandsHandle = _GenerateRaycastJOB.Schedule(_NBoids * _NRays, 8);
            JobHandle rayCastHandle = RaycastCommand.ScheduleBatch(_RayCastCommands, _ObstacleHits, 8, genCommandsHandle);

            _CorrectionForces.CopyFrom(_ZeroArray);

            _CollisionJOB._OldVelocity = _OldVelocities;
            JobHandle collisionHandle = _CollisionJOB.Schedule(_NBoids, 8, rayCastHandle);

            //! Boid grid hashing
            _GridToBoidsMap.Clear();

            HashBoidsToGirdJOB _HashBoidsJOB = new HashBoidsToGirdJOB()
            {
                _CubeSize = _SimArea.InnerCubeSize,
                _Divitions = _SimArea.Divitions,
                _OldPos = _OldPositions,
                _BoidToGridMap = _BoidToGridMap,
                _GridToBoidMap = _GridToBoidsMap.AsParallelWriter(),
                _AreaSize = _SimArea.Size
            };

            // Scheduling
            JobHandle hashBoidsHandle = _HashBoidsJOB.Schedule(_Positions.Length, 8);

            _AlignJOB._OldPosition = _OldPositions;
            _AlignJOB._OldVelocity = _OldVelocities;
            JobHandle alignBoidsHandle = _AlignJOB.Schedule(_NBoids, 8, hashBoidsHandle);

            //! Update
            _UpdateJOB._SystemOptions = _SystemOptions;
            _UpdateJOB.deltaTime = Time.deltaTime;
            _UpdateJOB._OldPosition = _OldPositions;
            _UpdateJOB._OldVelocity = _OldVelocities;
            _UpdateJOB._Position = _Positions;
            _UpdateJOB._Velocity = _Velocities;
            JobHandle updateHandle = _UpdateJOB.Schedule(_NBoids, 8, JobHandle.CombineDependencies(collisionHandle, alignBoidsHandle));

            updateHandle.Complete();

            //! Debug
            //var watch = new System.Diagnostics.Stopwatch();
            //watch.Reset();
            //watch.Start();

            //float deltaTime = Time.fixedDeltaTime;

            ////! Save old data
            //NativeArray<float3> oldPos = new NativeArray<float3>(_Positions, Allocator.TempJob);
            //NativeArray<float3> oldVel = new NativeArray<float3>(_Velocities, Allocator.TempJob);

            ////! Obstacle Findidng
            ////TODO: Recycle this JOB
            //GenerateRayCastCommandsJOB _GenerateCastJob = new GenerateRayCastCommandsJOB() {
            //    _Pos = _Positions,
            //    _HitMask = LayerMask.NameToLayer("BoidObs"),
            //    _RayCastCommands = _RayCastCommands,
            //    _VisDistance = _SystemOptions.ObstacleVision,
            //    _NumberOfBoids = _NOfBodies,
            //    _RayDirections = _RayDirections,
            //    _TransformMatrices = _Matrices
            //};

            //// Schedule both jobs
            //JobHandle genCastHandle = _GenerateCastJob.Schedule(_NOfBodies * _NumberOfRays, 8);
            //JobHandle rayCastHandle = RaycastCommand.ScheduleBatch(_RayCastCommands, _ObstacleHits, 8, genCastHandle);

            ////! Handle Obstacles
            //_CollisionJob._NewVel = _Velocities;
            //JobHandle collisionHandle = _CollisionJob.Schedule(_NOfBodies, 8, rayCastHandle);

            ////! Boid grid hashing
            //_GridToBoidsMap.Dispose();
            //_GridToBoidsMap = new NativeMultiHashMap<int, int>(_SimArea.Divitions.x* _SimArea.Divitions.y* _SimArea.Divitions.z, Allocator.TempJob);

            //HashBoidsToGirdJOB _HashBoidsJOB = new HashBoidsToGirdJOB()
            //{
            //    _CubeSize = _SimArea.InnerCubeSize,
            //    _Divitions = _SimArea.Divitions,
            //    _Pos = oldPos,
            //    _CubeToBoidMap = _GridToBoidsMap.AsParallelWriter(),
            //    _AreaSize = _SimArea.Size
            //};

            //// Scheduling
            //JobHandle hashBoidsHandle = _HashBoidsJOB.Schedule(_Positions.Length, 8);

            ////! Flock behaviour job
            //_FlockJOB._OldPos = oldPos;
            //_FlockJOB._OldVel = oldVel;
            //_FlockJOB._GridToBoidsMap = _GridToBoidsMap;
            //_FlockJOB._RayCastHits = _ObstacleHits;
            //_FlockJOB.deltaTime = deltaTime;
            //_FlockJOB._SystemOptions = _SystemOptions;
            //_FlockJOB.rand = new Unity.Mathematics.Random((uint)UnityEngine.Random.Range(1,15478));
            //_FlockJOB._VisDistance = _SystemOptions.ObstacleVision;

            //// Scheduling
            //JobHandle updateFlockHandle = _FlockJOB.Schedule(_SimArea.NumberOfCubes, 8, JobHandle.CombineDependencies(hashBoidsHandle, collisionHandle));

            ////! Simple update. If the above calculations take more time, they might be schedule every n frames, but this can still run every frame.
            //_UpdateJOB.deltaTime = deltaTime;
            //_UpdateJOB._OldVel = oldVel;
            //_UpdateJOB._SystemOptions = _SystemOptions;

            //// Scheduling
            //updateHandle = _UpdateJOB.Schedule(_NOfBodies, 8, updateFlockHandle);

            ////! Complete All Jobs
            //genCastHandle.Complete();
            //rayCastHandle.Complete();
            //hashBoidsHandle.Complete();
            //updateFlockHandle.Complete();
            //updateHandle.Complete();

            ////! Dispose temporary collections. This might be improved and may not need to be disposed every time, maybe some swapping with the current positions
            //oldPos.Dispose();
            //oldVel.Dispose();

            //// Debug
            //watch.Stop();

            //Debug.Log($"TIme ellapsed: {watch.Elapsed.TotalMilliseconds}");
        }

        public void GetMatrices(ref NativeArray<float4x4> matrices)
        {
            if (_Matrices == null)
                throw new System.Exception("Not initialized yet.");

            matrices = _Matrices;
        }

        public void DrawSystem()
        {
            Gizmos.color = Color.grey;
            Gizmos.DrawLine(_OldPositions[0], _OldPositions[0] + _OldVelocities[0]);

            Gizmos.color = Color.white;
            Gizmos.DrawWireSphere(_OldPositions[0], _SystemOptions.ObstacleVision);

            Gizmos.color = Color.magenta;
            Gizmos.DrawRay(_OldPositions[0], _CorrectionForces[0]);

            for (int i = 0; i < _RayDirections.Length; i++)
            {
                int index = i * _NBoids + 0;

                Gizmos.color = _ObstacleHits[index].collider == null ? Color.green : Color.red;

                if (index == 0)
                    Gizmos.color = _ObstacleHits[index].collider == null ? Color.blue : Color.cyan;

                Gizmos.DrawRay(_RayCastCommands[index].from, _RayCastCommands[index].direction * _SystemOptions.ObstacleVision);
            }

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

        public void GenerateSimulationWalls()
        {
            _QuadWallPrefab = (GameObject)Resources.Load("QuadWall");
            _SimulationAreaWalls = new List<GameObject>();

            var wall = GameObject.Instantiate(_QuadWallPrefab);
            wall.transform.position = new float3(_SimArea.Size.x * 0.5f, 0, 0);
            wall.transform.rotation = quaternion.LookRotation(new float3(1, 0, 0), Vector3.up);
            wall.transform.localScale = _SimArea.Size * 1.1f;
            _SimulationAreaWalls.Add(wall);

            wall = GameObject.Instantiate(_QuadWallPrefab);
            wall.transform.position = new float3(-_SimArea.Size.x * 0.5f, 0, 0);
            wall.transform.rotation = quaternion.LookRotation(new float3(-1, 0, 0), Vector3.up);
            wall.transform.localScale = _SimArea.Size * 1.1f;
            _SimulationAreaWalls.Add(wall);

            wall = GameObject.Instantiate(_QuadWallPrefab);
            wall.transform.position = new float3(0, _SimArea.Size.y * 0.5f, 0);
            wall.transform.rotation = quaternion.LookRotation(new float3(0, 1, 0), Vector3.forward);
            wall.transform.localScale = _SimArea.Size * 1.1f;
            _SimulationAreaWalls.Add(wall);

            wall = GameObject.Instantiate(_QuadWallPrefab);
            wall.transform.position = new float3(0, -_SimArea.Size.y * 0.5f, 0);
            wall.transform.rotation = quaternion.LookRotation(new float3(0, -1, 0), Vector3.forward);
            wall.transform.localScale = _SimArea.Size * 1.1f;
            _SimulationAreaWalls.Add(wall);

            wall = GameObject.Instantiate(_QuadWallPrefab);
            wall.transform.position = new float3(0, 0, _SimArea.Size.z * 0.5f);
            wall.transform.rotation = quaternion.LookRotation(new float3(0, 0, 1), Vector3.up);
            wall.transform.localScale = _SimArea.Size * 1.1f;
            _SimulationAreaWalls.Add(wall);

            wall = GameObject.Instantiate(_QuadWallPrefab);
            wall.transform.position = new float3(0, 0, -_SimArea.Size.z * 0.5f);
            wall.transform.rotation = quaternion.LookRotation(new float3(0, 0, -1), Vector3.up);
            wall.transform.localScale = _SimArea.Size * 1.1f;
            _SimulationAreaWalls.Add(wall);
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

        private void Swap<T>(ref T array1, ref T array2)
        {
            T tmp = array1;
            array1 = array2;
            array2 = tmp;
        }
    }
}
