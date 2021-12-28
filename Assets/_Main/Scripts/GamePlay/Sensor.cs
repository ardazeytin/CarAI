using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Sensor : MonoBehaviour
{
    [SerializeField] private Color gizmoColor = Color.green;
    [SerializeField] private Vector3 sensorGizmoSize = new Vector3(0.1f, 0.1f, 0.1f);

    private void OnDrawGizmos()
    {
        Gizmos.color = gizmoColor;
        Gizmos.DrawCube(transform.position, sensorGizmoSize);
    }
}
