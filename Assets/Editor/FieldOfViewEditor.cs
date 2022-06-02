using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof (FieldOfView))]
public class FieldOfViewEditor : Editor
{
    private void OnSceneGUI()
    {
        FieldOfView fieldOfView = (FieldOfView)target;
        Handles.color = Color.white;
        Handles.DrawWireArc(fieldOfView.transform.position, Vector3.up, Vector3.forward, 360, fieldOfView.ViewRadius);

        Vector3 viewAngleA = fieldOfView.DirFromAngle(-fieldOfView.ViewAngle / 2, false);
        Vector3 viewAngleB = fieldOfView.DirFromAngle(fieldOfView.ViewAngle / 2, false);

        Handles.DrawLine(fieldOfView.transform.position, fieldOfView.transform.position + viewAngleA * fieldOfView.ViewRadius);
        Handles.DrawLine(fieldOfView.transform.position, fieldOfView.transform.position + viewAngleB * fieldOfView.ViewRadius);

        Handles.color = Color.red;
        foreach(Transform visibleTarget in fieldOfView.VisibleTargets)
        {
            Handles.DrawLine(fieldOfView.transform.position, visibleTarget.position);
        }
    }
}
