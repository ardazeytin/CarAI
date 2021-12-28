using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;

public class PathController : MonoBehaviour
{
    [Header("Settings")]
    public PathType pathType = PathType.Circuit;
    public enum PathType { Circuit, Sprint }

    [Header("Debug")]
    public Color lineColor;
    //[SerializeField] private GizmoType gizmoType = GizmoType.Pickable;

    private List<Transform> nodes = new List<Transform>();

    void OnDrawGizmos()
    {
        Gizmos.color = lineColor;


        Transform[] pathTransforms = GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for(int i = 0; i < pathTransforms.Length; i++)
        {
            if(pathTransforms[i] != transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }

        for(int i = 0; i < nodes.Count; i++)
        {
            Vector3 currentNode = nodes[i].position;
            Vector3 previousNode = Vector3.zero;

            if (i > 0)
            {
                previousNode = nodes[i - 1].position;
            }
            else if(i == 0 && nodes.Count > 1)
            {
                previousNode = nodes[nodes.Count - 1].position;
            }

            if (pathType == PathType.Sprint)
            {
                if (previousNode != nodes[nodes.Count - 1].position)
                {
                    Gizmos.DrawLine(previousNode, currentNode);
                }
            }
            else if (pathType == PathType.Circuit)
            {
                Gizmos.DrawLine(previousNode, currentNode);
            }

            if (currentNode == nodes[0].position)
            {
                //First node
                Gizmos.DrawWireCube(currentNode, new Vector3(0.5f, 1.5f, 0.5f));
            }
            else if (currentNode == nodes[nodes.Count - 1].position)
            {
                //last node
                Gizmos.DrawWireCube(currentNode, new Vector3(0.5f,1.5f,0.5f));
            }
            else
            {
                //other node
                Gizmos.color = lineColor;
                Gizmos.DrawWireSphere(currentNode, 0.3f);
            }
        }
    }
}
