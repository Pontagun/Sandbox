using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Rot : MonoBehaviour
{
    float m_MyX, m_MyY, m_MyZ;
    //These are the Sliders that set the rotation. Remember to assign these in the Inspector
    public Slider m_SliderX, m_SliderY, m_SliderZ;
    //These are the Texts that output the current value of the rotations. Remember to assign these in the Inspector
    public Text m_TextX, m_TextY, m_TextZ;
    // Start is called before the first frame update
    //Change the Quaternion values depending on the values of the Sliders
    private static Quaternion Change(float x, float y, float z)
    {
        //Return the new Quaternion
        return new Quaternion(x, y , z, 1);
    }

    void Start()
    {
    }

    void Update()
    {
        // transform.rotation = new Quaternion(0.000105f,0.024050f,0.002935f,0.999706f);
        // transform.rotation = new Quaternion(0.0f,0f,0f,0.0f);
        // transform.rotation = new Quaternion(-0.000910f,0.000445f,-0.000769f,0.999999f);
        // transform.rotation = new Quaternion(-0.003160f,0.006250f,0.708807f,0.705368f);
        // transform.rotation = new Quaternion(0.695319f,0.041790f,-0.017583f,0.717270f);
        // transform.rotation = new Quaternion(-0.047697f,0.765433f,0.007362f,0.641704f);
        // transform.rotation = new Quaternion(0.695988f,-0.188759f,0.211785f,0.659635f);

        transform.rotation = new Quaternion(-0.20f,0.9f,-.2f,0.25f);
        // 0.065667,0.088526,0.708230,0.697324
        // 0.690284,0.103993,-0.078927,0.711663
        // -0.050110,0.806821,-0.002128,0.588664
        // 0.707012,-0.134038,0.161637,0.675308
    }
}
