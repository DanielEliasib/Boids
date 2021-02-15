using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AL.BoidSystem;
using Unity.Mathematics;

public class BoidManager : MonoBehaviour
{
    private BoidSystem _System;
    private bool _Init;


    // Start is called before the first frame update
    void Start()
    {
        _Init = false;
        _System = new BoidSystem(8*100, new SimulationArea(new int3(13,13,13), float3.zero, new float3(20,20,20)));
        _Init = true;
    }

    // Update is called once per frame
    void Update()
    {
        _System.UpdateSystem();
    }

    private void OnDestroy()
    {
        _System.Dispose();
    }

    private void OnDrawGizmos()
    {
        if (_Init)
        {
            _System.DrawSystem();
            _System.DrawSimulationArea();
        }
    }
}
