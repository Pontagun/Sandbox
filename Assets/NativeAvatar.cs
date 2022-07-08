using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NuitrackSDK;

public class NativeAvatar : MonoBehaviour
{
    string message = "";

    public nuitrack.JointType[] typeJoint;
    GameObject[] CreatedJoint;
    public GameObject PrefabJoint;

    // Start is called before the first frame update
    void Start()
    {
        CreatedJoint = new GameObject[typeJoint.Length];
        for (int q = 0; q < typeJoint.Length; q++)
        {
            CreatedJoint[q] = Instantiate(PrefabJoint);
            CreatedJoint[q].transform.SetParent(transform);
        }
        Debug.Log(typeJoint.Length);
        message = "Skeleton created";
    }

    // Update is called once per frame
    void Update()
    {
        void Update()
        {
            if (NuitrackManager.Users.Current != null && NuitrackManager.Users.Current.Skeleton != null)
            {
                message = "User found";

                for (int q = 0; q < typeJoint.Length; q++)
                {
                    UserData.SkeletonData.Joint joint = NuitrackManager.Users.Current.Skeleton.GetJoint(typeJoint[q]);
                    CreatedJoint[q].transform.localPosition = joint.Position;
                }
            }
            else
            {
                message = "User not found";
            }
        }
    }

    void OnGUI()
    {
        GUI.color = Color.red;
        GUI.skin.label.fontSize = 50;
        GUILayout.Label(message);
    }
}
