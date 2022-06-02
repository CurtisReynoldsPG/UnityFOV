using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FieldOfView : MonoBehaviour
{
    [SerializeField] LayerMask _targetMask, _obstacleMask;

    [SerializeField] private float _viewRadius;
    [Range(0, 360), SerializeField] private float _viewAngle;

    [SerializeField] private int _maxTargets = 0;

    private List<Transform> _visibleTargets = new List<Transform>();
    public List<Transform> VisibleTargets => _visibleTargets;

    private int _targetsHit = 10;

    private Collider[] _targetsInViewRadius;

    public float ViewRadius => _viewRadius;
    public float ViewAngle => _viewAngle;

    void Awake()
    {
        _targetsInViewRadius = new Collider[_maxTargets];
    }

    private void Start()
    {
        StartCoroutine(FindTargetsWithDelay(.1f));
    }

    public Vector3 DirFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
        if(!angleIsGlobal)
        {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad),0,Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    private void FindVisibleTargets()
    {
        _visibleTargets.Clear();
        _targetsHit = 0;


        _targetsHit = Physics.OverlapSphereNonAlloc(transform.position, _viewRadius,_targetsInViewRadius,_targetMask);

        for (int i = 0; i < _targetsHit; i++)
        {
            Transform targetTransform = _targetsInViewRadius[i].transform;
            Vector3 dirToTarget = (targetTransform.position - transform.position).normalized;
            if (Vector3.Angle(transform.forward, dirToTarget) < _viewAngle / 2)
            {
                float distanceToTarget = Vector3.Distance(transform.position, targetTransform.position);

                if (!Physics.Raycast(transform.position, dirToTarget, distanceToTarget, _obstacleMask))
                {
                    _visibleTargets.Add(targetTransform);
                }
            }
        }
    }
    IEnumerator FindTargetsWithDelay(float delay)
    {
        while (true)
        {
            yield return new WaitForSeconds(delay);
            FindVisibleTargets();
        }
    }
    private void OnDrawGizmos()
    {
        Vector3 viewAngleA = DirFromAngle(-ViewAngle / 2, false);
        Vector3 viewAngleB = DirFromAngle(ViewAngle / 2, false);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + viewAngleA * ViewRadius);
        Gizmos.DrawLine(transform.position, transform.position + viewAngleB * ViewRadius);

        for (int i = 0; i < _targetsHit; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, _targetsInViewRadius[i].transform.position);
        }

        foreach (Transform target in _visibleTargets)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, target.position);
        }
    }

}
