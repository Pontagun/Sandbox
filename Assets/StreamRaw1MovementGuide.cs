using UnityEngine;
using System;
using System.Collections;
using System.IO.Ports;
using UnityEditor;
using System.Text;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


public class StreamRaw1MovementGuide : MonoBehaviour
{

    private IEnumerator coroutine;
    SensorConnection ssCon;
    // Connect to the serial port the 3-Space Sensor is connected to

    // public static SerialPort sp0 = new SerialPort("\\\\.\\COM3", 115200, Parity.None, 8, StopBits.One);
    // Print-to-file variables
    SerialPort sp0;
    public string filenameNo = "";
    private static string FileStr0;
    private static string FileStr1;
    private static string FileStrMark;
    private int rotCount = 0;
    private int markCount = 0;
    private bool handRotated = true;
    private float k = 0.0f;
    private GUIStyle guiStyle = new GUIStyle();
    List<Quaternion> Qref = new List<Quaternion>();
    List<Vector3> uPref = new List<Vector3>();
    List<Vector3> Pref = new List<Vector3>();

    // Command packet for getting the filtered tared orientation as a quaternion
    // {header byte, command byte, [data bytes], checksum byte}
    // checksum = (command byte + data bytes) % 256

   

    public static bool EndOfSimulation = false;

    public static Quaternion HandOrientation;
    public static Vector3 HandPosition;

    // ---------- Algorithm Variables ---------- //

    private double CurrentTime = 0.00;
    private double PreviousTime = 0.00;
    private float SamplingTime;
    private int N = 0;
    private static int BuffSize = 3;
    private float B_SCALING = 1.0f;
    private float alpWeight = 0.25f;
    private float muWeight = 0.25f;



    private float Stillness0;
    private Vector3 Gyro0, Accelero0, Magneto0;
    private Quaternion IMUQuat0;
    private Vector3[] GyroBuff0 = new Vector3[BuffSize];
    private Vector3[] AcceleroBuff0 = new Vector3[BuffSize];
    private Vector3[] MagnetoBuff0 = new Vector3[BuffSize];
    private float[] StillnessBuff0 = new float[BuffSize];
    private Vector3 GyroAvg0 = new Vector3();
    private Vector3 AcceleroAvg0 = new Vector3();
    private Vector3 MagnetoAvg0 = new Vector3();
    private float StillnessAvg0 = new float();
    private float alpha0 = new float();
    private float prevAlphaX0 = 1.0f;
    private float prevAlphaY0 = 1.0f;
    private float thisAlphaX0 = new float();
    private float thisAlphaY0 = new float();
    private Vector3 Bias0 = new Vector3(0, 0, 0);
    private Vector3 fixedBias0 = new Vector3(-0.008081f, -0.002751f, -0.008184f); // <------------------------- Fixed single bias values -------------------------------
    private Vector3 BiasBuff0 = new Vector3(0, 0, 0);
    private int trigcount0 = 0;
    private Vector3 UnbiasedGyro0 = new Vector3(0, 0, 0);
    private Vector3 UnbiasedGyroWithfixedBias0 = new Vector3(0, 0, 0);
    private Quaternion w0; // Pure Quaternion
    private Quaternion wfixed0; // Pure Quaternion
    private Quaternion dqG0;
    private Quaternion dqGfixed0;
    private Quaternion qG0 = new Quaternion(0, 0, 0, 1);
    private Quaternion qGfixed0 = new Quaternion(0, 0, 0, 1);
    private Quaternion dqGA0;
    private Quaternion qGA0 = new Quaternion(0, 0, 0, 1);
    private Quaternion dqGM0;
    private Quaternion qGM0 = new Quaternion(0, 0, 0, 1);
    private Quaternion A_int0 = new Quaternion();
    private Quaternion M_int0 = new Quaternion();
    private Quaternion a40 = new Quaternion(0, 0, 0, 1);
    private Vector3 a30 = new Vector3();
    private Quaternion m40 = new Quaternion(0, 0, 0, 1);
    private Vector3 m30 = new Vector3();
    public static Quaternion qOUT0 = new Quaternion(0, 0, 0, 1);
    public static Quaternion qOUT1 = new Quaternion(0, 0, 0, 1);

    private Quaternion IMUQuat0Hist;
    private Quaternion Q0Hist;
    private Quaternion Q1Hist;

    private Quaternion tempQ;
    private Quaternion tempQ0;
    private Quaternion tempQ1;

    private Quaternion Dqout0 = new Quaternion(0, 0, 0, 1);
    private Quaternion Dqout1 = new Quaternion(0, 0, 0, 1);

    public static Quaternion dIMUQuat;
    public static Quaternion dQ0;
    public static Quaternion dQ1;

    private Vector3 EulerFixed0;
    private Vector3 EulerKalman0;
    private Vector3 EulerGMV1;
    private Vector3 EulerGMV0;

    private Quaternion qGfixed0e;
    private Quaternion IMUQuat0e;
    public static Quaternion qOUT0e;
    public static Quaternion qOUT1e;

    private Vector3 EulerFixed0e;
    private Vector3 EulerKalman0e;
    private Vector3 EulerGMV0e;
    private Vector3 EulerGMV1e;

    private Vector3 PosE;

    private static int VoxNO = 200;
    private static float[,,] MU = new float[VoxNO, VoxNO, VoxNO];
    //float MU[VoxNO][VoxNO] [VoxNO] = {};

    float TA = 0.80f; //Alpha Threshold
    float TM = 0.80f; //MU Threshold

    Quaternion qGpost, mGA4;
    Vector3 mGA3;
    float cosGamma, norm_MagAvg, norm_mGA3, temp_mu, gg, gl;
    static float thisMu = 0.0f, prevMu = 0.0f, thisTempMu = 0.0f, prevTempMu = 0.0f, mu0 = 0.0f;
    int Stage;
    int ma = 1;
    int mt = 2;

    static int LocateVoxX = 0;
    static int LocateVoxY = 0;
    static int LocateVoxZ = 0;

    float prevMuX = 0.0f;
    float prevMuY = 0.0f;

    //string[] step = { "Start at initial position", "Move to position 1", "Move to position 2", "Move to initial position" };
    string[] step = { "", "", "", "" };
    Stopwatch stopwatch = new Stopwatch();
    String execTime = @"Assets/execTime.txt";
    int sample = 0;
    ArrayList exectimeStart = new ArrayList();
    ArrayList exectimeStop = new ArrayList();
    TimeUtils tu = new TimeUtils();

    void Start()
    {
        ssCon = new SensorConnection();
        sp0 =  ssCon.ConnectionInit();
        initQref();
    }

    void Update()
    {
        if (!EndOfSimulation)
        {
            // A quaternion consists of 4 floats which is 16 bytes
            byte[] read_bytes0 = new byte[56];   // <----------------- No. of Bytes
                                                 // Mono, for some reason, seems to randomly fail on the first read after a write so we must loop
                                                 // through to make sure the bytes are read and Mono also seems not to always read the amount asked
                                                 // so we must also read one byte at a time
            int read_counter = 100;
            int byte_idx0 = 0;

            PreviousTime = CurrentTime;
            CurrentTime += 1 * Time.deltaTime;

            SamplingTime = (float)(CurrentTime - PreviousTime);

            

            while (read_counter > 0)
            {

                try
                {
                    byte_idx0 += sp0.Read(read_bytes0, byte_idx0, 1);
                }
                catch
                {
                    // Failed to read from serial port
                }
                if (byte_idx0 == 56)
                { // <----------------- No. of Bytes
                    break;
                }
                if (read_counter <= 0)
                {
                    throw new System.Exception("Failed to read quaternion from port too many times." +
                        " This could mean the port is not open or the Mono serial read is not responding.");
                }
                --read_counter;
            }

            sensorByteToFloat(read_bytes0);



            DateTime a = DateTime.Now;
            String nnstr = a.ToString("ffffff");
            String fnnstr = a.ToString("HH:mm:ss.ffffff");
            
            //testOng();
            testNan();
            
            DateTime b = DateTime.Now;
            String nnstp = b.ToString("ffffff");
            String fnnstp = b.ToString("HH:mm:ss.ffffff");

            //System.IO.File.AppendAllText(execTime, System.String.Format("{0},{1},{2},{3},{4},{5}\n", sample, fnnstr, fnnstp, nnstp, nnstr, (Convert.ToInt64(nnstp) - Convert.ToInt64(nnstr))));

            sensorInterperter();
            if (Input.GetKey("escape"))
            {
                //for (int i = 0; i < exectimeStart.Count; i++)
                //{
                //    System.IO.File.AppendAllText(execTime, System.String.Format("{0}, {1}, {2}, {3}\n", i, exectimeStart[i], exectimeStop[i], (exectimeStop[i] - exectimeStart[i])));
                //}

                long t1 = tu.GetNanoseconds();

                Thread.Sleep(1000);

                long t2 = tu.GetNanoseconds();
                long dt = t2 - t1;

                Debug.Log(dt);

                sp0.Close();
                print("All ports are closed!");
                EndOfSimulation = true;
            }
        }

        if (rotCount != 0)
        {
            HandOrientation = Quaternion.Slerp(Qref[rotCount - 1], Qref[rotCount], k);
            // HandPosition = Vector3.Lerp(uPref[rotCount - 1], uPref[rotCount], k);
            if (k < 1)
            {
                k += 0.05f; // Set speed rotation animation
            }
        }
        else
        {
            HandOrientation = Qref[rotCount];
            // HandPosition = uPref[rotCount];
        }

        //print ("x=" + HandOrientation.x + " y=" + HandOrientation.y + " z=" + HandOrientation.z + " w=" + HandOrientation.w);
        this.transform.rotation = HandOrientation;
        // this.transform.position = HandPosition;

    }

    void initQref()
    {
        //FileStr0 = "Assets/Raw1Recordings/rec" + filenameNo + "GMV0.txt";
        //FileStr1 = "Assets/Raw1Recordings/rec" + filenameNo + "GMV1.txt";
        //FileStrMark = "Assets/Raw1Marks/mark" + filenameNo + ".txt";

        IMUQuat0Hist = new Quaternion(0, 0, 0, 0);
        Q0Hist = new Quaternion(0, 0, 0, 0);
        Q1Hist = new Quaternion(0, 0, 0, 0);

        // Start box at dock.
        Qref.Add(Quaternion.Euler(0, 0, 0));
        // Move hand to Pos1
        // Non mag distortion
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(0, 0, 90));
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(90, 0, 0));
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(0, 90, 0)); // Check y
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(90, 0, 45)); //
        Qref.Add(Quaternion.Euler(0, 0, 0));
        // Move hand to Pos2
        // mag distortion
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(0, 0, 90));
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(90, 0, 0));
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(0, 90, 0)); // Check y
        Qref.Add(Quaternion.Euler(0, 0, 0));
        Qref.Add(Quaternion.Euler(90, 0, 45)); //
        Qref.Add(Quaternion.Euler(0, 0, 0));
        // Move hand to Dock
        Qref.Add(Quaternion.Euler(0, 0, 0));

        //Qref.Add(Quaternion.Euler(-90, 0, 0));
        //Qref.Add(Quaternion.Euler(0, 0, 0));
        //Qref.Add(Quaternion.Euler(-45, 90, -90));
        //Qref.Add(Quaternion.Euler(-45, 90, -90));
        //Qref.Add(Quaternion.Euler(0, 0, 0));

        //Qref.Add(Quaternion.Euler(0, 0, 0));
        //Qref.Add(Quaternion.Euler(0, 0, 90));
        //Qref.Add(Quaternion.Euler(0, 0, 0));
        //Qref.Add(Quaternion.Euler(0, 90, 0));
        //Qref.Add(Quaternion.Euler(0, 0, 0));
        //Qref.Add(Quaternion.Euler(-90, 0, 0));
        //Qref.Add(Quaternion.Euler(0, 0, 0));
        //Qref.Add(Quaternion.Euler(-45, 90, -90));
        //Qref.Add(Quaternion.Euler(-45, 90, -90));
        //Qref.Add(Quaternion.Euler(0, 0, 0));

        //uPref.Add(new Vector3(0.0f, 0.0f, -0.504f));
        //uPref.Add(new Vector3(0.0f, 0.0f, -0.504f));
        //uPref.Add(new Vector3(0.0f, 0.0f, -0.504f));
        //uPref.Add(new Vector3(0.0f, 0.0f, 0.0f)); //?????????????????????????????????????????
        //uPref.Add(new Vector3(0.0f, 0.0f, -0.504f));
        //uPref.Add(new Vector3(0.0f, 0.0f, 0.0f)); //?????????????????????????????????????????
        //uPref.Add(new Vector3(0.0f, 0.0f, -0.504f));
        //uPref.Add(new Vector3(0.0f, 0.0f, 0.0f)); //?????????????????????????????????????????
        //uPref.Add(new Vector3(-0.3625f, -0.3625f, 0.0f)); //?????????????????????????????????????????
        //uPref.Add(new Vector3(0.0f, 0.0f, -0.504f));

        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));
        //Pref.Add(new Vector3(-0.12f, -0.12f, -0.52f));
        //Pref.Add(new Vector3(0.0f, 0.0f, -0.52f));

        // Set the read/write timeoutsS
    }

    void sensorByteToFloat(byte[] read_bytes0)
    {
        // Convert bytes to floats
        Stillness0 = bytesToFloat(read_bytes0, 0);
        Gyro0.x = bytesToFloat(read_bytes0, 4);
        Gyro0.y = bytesToFloat(read_bytes0, 8);
        Gyro0.z = bytesToFloat(read_bytes0, 12);
        Accelero0.x = bytesToFloat(read_bytes0, 16);
        Accelero0.y = bytesToFloat(read_bytes0, 20);
        Accelero0.z = bytesToFloat(read_bytes0, 24);
        Magneto0.x = bytesToFloat(read_bytes0, 28);
        Magneto0.y = bytesToFloat(read_bytes0, 32);
        Magneto0.z = bytesToFloat(read_bytes0, 36);
        IMUQuat0.x = bytesToFloat(read_bytes0, 40);
        IMUQuat0.y = bytesToFloat(read_bytes0, 44);
        IMUQuat0.z = bytesToFloat(read_bytes0, 48);
        IMUQuat0.w = bytesToFloat(read_bytes0, 52);
    }
    // Helper function for taking the bytes read from the 3-Space Sensor and converting them into a float
    float bytesToFloat(byte[] raw_bytes, int offset)
    {
        byte[] big_bytes = new byte[4];
        big_bytes[0] = raw_bytes[offset + 3];
        big_bytes[1] = raw_bytes[offset + 2];
        big_bytes[2] = raw_bytes[offset + 1];
        big_bytes[3] = raw_bytes[offset + 0];
        return BitConverter.ToSingle(big_bytes, 0);
    }

    Quaternion myQuatConj(Quaternion q)
    {

        Quaternion q_result = new Quaternion(-1.0f * q.x, -1.0f * q.y, -1.0f * q.z, q.w);
        return q_result;
    }

    Quaternion myQuatIntegrate(Quaternion dq, Quaternion q, float dt)
    {

        Quaternion omega = dq * myQuatConj(q);
        omega = new Quaternion(2.0f * omega.x, 2.0f * omega.y, 2.0f * omega.z, 2.0f * omega.w);
        omega = new Quaternion((omega.x * dt) / 2.0f, (omega.y * dt) / 2.0f, (omega.z * dt) / 2.0f, (omega.w * dt) / 2.0f);
        float omega_norm2 = Mathf.Sqrt(Mathf.Pow(omega.x, 2) + Mathf.Pow(omega.y, 2) + Mathf.Pow(omega.z, 2));
        Quaternion exp = new Quaternion();

        if (omega_norm2 != 0)
        {

            exp.x = Mathf.Exp(omega.w) * (Mathf.Sin(omega_norm2) / omega_norm2) * omega.x;
            exp.y = Mathf.Exp(omega.w) * (Mathf.Sin(omega_norm2) / omega_norm2) * omega.y;
            exp.z = Mathf.Exp(omega.w) * (Mathf.Sin(omega_norm2) / omega_norm2) * omega.z;
            exp.w = Mathf.Exp(omega.w) * Mathf.Cos(omega_norm2);
        }
        else
        {

            exp.x = Mathf.Exp(omega.w) * omega.x;
            exp.y = Mathf.Exp(omega.w) * omega.y;
            exp.z = Mathf.Exp(omega.w) * omega.z;
            exp.w = Mathf.Exp(omega.w) * Mathf.Cos(omega_norm2);
        }

        Quaternion q_result = exp * q;

        return q_result;
    }

    Quaternion myQuatNormalize(Quaternion q)
    {

        float q_norm = Mathf.Sqrt((Mathf.Pow(q.x, 2) + Mathf.Pow(q.y, 2) + Mathf.Pow(q.z, 2) + Mathf.Pow(q.w, 2)));
        Quaternion q_result = new Quaternion(q.x / q_norm, q.y / q_norm, q.z / q_norm, q.w / q_norm);

        return q_result;
    }

    //void OnGUI()
    void sensorInterperter()
    {
        //GUIStyle buttonGUIStyle = new GUIStyle("button");
        //buttonGUIStyle.normal.textColor = Color.green;
        //buttonGUIStyle.hover.textColor = Color.green;
        //buttonGUIStyle.fontStyle = FontStyle.Bold;

        //GUILayout.BeginArea(new Rect(10, Screen.height - 40, 200, 200));
        //GUILayout.BeginVertical("box");
        //if (GUILayout.Button("Close ALL Ports"))
        //{
        //    sp0.Close();
        //    print("COM4 is closed!");
        //    EndOfSimulation = true;
        //    UnityEditor.EditorApplication.isPlaying = false;
        //}
        //GUILayout.EndVertical();
        //GUILayout.EndArea();

        //GUILayout.BeginArea(new Rect(Screen.width / 2 - 100, 40, 300, 200));
        //if (!EndOfSimulation)
        //{
        //    // GUILayout.Label("Now Recording... Subject" + filenameNo + "\n" + markCount.ToString() + "/" + Qref.Count.ToString() + " orientation(s) marked");
        //}
        //else
        //{
        //    // GUILayout.Label("Recording Completed for Subject" + filenameNo);
        //}
        //GUILayout.EndArea();

        //if (markCount < Qref.Count)
        //{
        //    if (handRotated)
        //    {
        if (true)
        {
            if (true)
            {
                //GUILayout.BeginArea(new Rect(Screen.width - 210, Screen.height - 40, 200, 200));
                //GUILayout.BeginVertical("box");
                //if (GUILayout.Button("Mark this position & orientation"))
                if (true)
                {
                    //PosE.x = Pref [rotCount].x - OptitrackRigidBodyManager.instance.omPositions [0].x;
                    //PosE.y = Pref [rotCount].y - OptitrackRigidBodyManager.instance.omPositions [0].y;
                    //PosE.z = Pref [rotCount].z - OptitrackRigidBodyManager.instance.omPositions [0].z;
                    //PosE.x = OptitrackStreamingClient.markerX;
                    //PosE.y = OptitrackStreamingClient.markerY;
                    //PosE.z = OptitrackStreamingClient.markerZ;

                    /*
                    EulerFixed0e = qGfixed0.eulerAngles;
                    EulerKalman0e = IMUQuat0.eulerAngles;
                    EulerGMV0e = qOUT0.eulerAngles;
                    EulerGMV1e = qOUT1.eulerAngles;*/

                    qGfixed0e = myQuatNormalize(myQuatConj(Qref[rotCount]) * qGfixed0);
                    IMUQuat0e = myQuatNormalize(myQuatConj(Qref[rotCount]) * IMUQuat0);
                    qOUT0e = myQuatNormalize(myQuatConj(Qref[rotCount]) * qOUT0);
                    qOUT1e = myQuatNormalize(myQuatConj(Qref[rotCount]) * qOUT1);

                    EulerFixed0e = qGfixed0e.eulerAngles;
                    EulerKalman0e = IMUQuat0e.eulerAngles;
                    EulerGMV0e = qOUT0e.eulerAngles;
                    EulerGMV1e = qOUT1e.eulerAngles;


                    if (EulerFixed0e.x > 180.0)
                        EulerFixed0e.x = -(360.0f - EulerFixed0e.x);
                    if (EulerFixed0e.y > 180.0)
                        EulerFixed0e.y = -(360.0f - EulerFixed0e.y);
                    if (EulerFixed0e.z > 180.0)
                        EulerFixed0e.z = -(360.0f - EulerFixed0e.z);

                    if (EulerKalman0e.x > 180.0)
                        EulerKalman0e.x = -(360.0f - EulerKalman0e.x);
                    if (EulerKalman0e.y > 180.0)
                        EulerKalman0e.y = -(360.0f - EulerKalman0e.y);
                    if (EulerKalman0e.z > 180.0)
                        EulerKalman0e.z = -(360.0f - EulerKalman0e.z);

                    if (EulerGMV0e.x > 180.0)
                        EulerGMV0e.x = -(360.0f - EulerGMV0e.x);
                    if (EulerGMV0e.y > 180.0)
                        EulerGMV0e.y = -(360.0f - EulerGMV0e.y);
                    if (EulerGMV0e.z > 180.0)
                        EulerGMV0e.z = -(360.0f - EulerGMV0e.z);

                    if (EulerGMV1e.x > 180.0)
                        EulerGMV1e.x = -(360.0f - EulerGMV1e.x);
                    if (EulerGMV1e.y > 180.0)
                        EulerGMV1e.y = -(360.0f - EulerGMV1e.y);
                    if (EulerGMV1e.z > 180.0)
                        EulerGMV1e.z = -(360.0f - EulerGMV1e.z);

                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("T{0},", CurrentTime));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},", Mathf.Abs(EulerFixed0e.x), Mathf.Abs(EulerFixed0e.y), Mathf.Abs(EulerFixed0e.z)));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},", Mathf.Abs(EulerKalman0e.x), Mathf.Abs(EulerKalman0e.y), Mathf.Abs(EulerKalman0e.z)));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},", Mathf.Abs(EulerGMV0e.x), Mathf.Abs(EulerGMV0e.y), Mathf.Abs(EulerGMV0e.z)));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2}", Mathf.Abs(EulerGMV1e.x), Mathf.Abs(EulerGMV1e.y), Mathf.Abs(EulerGMV1e.z)));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format(",---,"));
                    ////System.IO.File.AppendAllText(FileStrMark, System.String.Format("C {0},{1},{2}\n", PosE.x, PosE.y, PosE.z));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},", qGfixed0e.x, qGfixed0e.y, qGfixed0e.z, qGfixed0e.w));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},", IMUQuat0e.x, IMUQuat0e.y, IMUQuat0e.z, IMUQuat0e.w));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},", qOUT0e.x, qOUT0e.y, qOUT0e.z, qOUT0e.w));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3}", qOUT1e.x, qOUT1e.y, qOUT1e.z, qOUT1e.w));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format(",---,"));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},", qGfixed0.x, qGfixed0.y, qGfixed0.z, qGfixed0.w));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},", IMUQuat0.x, IMUQuat0.y, IMUQuat0.z, IMUQuat0.w));//Quaternion of KF
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},", qOUT0.x, qOUT0.y, qOUT0.z, qOUT0.w));//Quaternion of GMV-S
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3}\n", qOUT1.x, qOUT1.y, qOUT1.z, qOUT1.w));//Quaternion of GMV-D

                    Dqout0 = qOUT0;
                    Dqout1 = qOUT1;

                    tempQ.y = IMUQuat0.y;
                    IMUQuat0.y = -IMUQuat0.z;
                    tempQ.z = IMUQuat0.z;
                    IMUQuat0.z = tempQ.y;
                    dIMUQuat = Quaternion.Slerp(IMUQuat0Hist, IMUQuat0, 1);

                    tempQ0.y = Dqout0.y;
                    Dqout0.y = -Dqout0.z;
                    tempQ0.z = Dqout0.z;
                    Dqout0.z = tempQ0.y;
                    dQ0 = Quaternion.Slerp(Q0Hist, Dqout0, 1);

                    tempQ1.y = Dqout1.y;
                    Dqout1.y = -Dqout1.z;
                    tempQ1.z = Dqout1.z;
                    Dqout1.z = tempQ1.y;
                    dQ1 = Quaternion.Slerp(Q1Hist, Dqout1, 1);


                    IMUQuat0Hist = IMUQuat0;
                    Q0Hist = Dqout0;
                    Q1Hist = Dqout1;

                    //    System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15} \n"
                    //, qGfixed0.x, qGfixed0.y, qGfixed0.z, qGfixed0.w, IMUQuat0.x, IMUQuat0.y, IMUQuat0.z, IMUQuat0.w, qOUT0e.x, qOUT0e.y, qOUT0e.z, qOUT0e.w, qOUT1.x, qOUT1.y, qOUT1.z, qOUT1.w));
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11} \n", Mathf.Abs(EulerFixed0e.x), Mathf.Abs(EulerFixed0e.y), Mathf.Abs(EulerFixed0e.z)
                    //System.IO.File.AppendAllText(FileStrMark, System.String.Format("{0},{1}\n", DoubleS.alpha0, DoubleS.alpha0, qOUT0e.z, qOUT0e.w));
                    markCount++;
                    //print("Orientation marked (" + markCount.ToString() + "/" + Qref.Count.ToString() + ")");
                    handRotated = false;
                }
                //GUILayout.EndVertical();
                //GUILayout.EndArea();
            }
            else
            {

                //GUILayout.BeginArea(new Rect(Screen.width - 210, Screen.height - 40, 200, 200));
                //GUILayout.BeginVertical("box");
                //if (GUILayout.Button("Show Hand Sequence", buttonGUIStyle))
                //{

                //    print("Hand Model is rotating");
                //    rotCount++;
                //    k = 0.0f;
                //    handRotated = true;
                //}
                //GUILayout.EndVertical();
                //GUILayout.EndArea();
            }
            //guiStyle = GUI.skin.GetStyle("Label");
            //guiStyle.fontSize = 30; //change the font size
            //guiStyle.alignment = TextAnchor.UpperCenter;

            /*                          Moving box by order.                  */
            //    float movePosSp = 0.05f; // Positon moving speed.
            //    if (rotCount == 0)
            //    {
            //        GUI.Label(new Rect(Screen.width / 2 - 200, 40, 400, 100), step[0], guiStyle);
            //    }
            //    else if (rotCount == 1)
            //    {
            //        Vector3 newPosition = transform.position; // We store the current position

            //        if ((int)transform.position.z != -3)
            //        {
            //            newPosition.z = newPosition.z - movePosSp;
            //            transform.position = newPosition; // We pass it back
            //        }

            //        GUI.Label(new Rect(Screen.width / 2 - 200, 40, 400, 100), step[1], guiStyle);
            //    }
            //    else if (rotCount == 10)
            //    {
            //        Vector3 newPosition = transform.position; // We store the current position
            //        if ((int)transform.position.x != -3)
            //        {
            //            newPosition.x = newPosition.x - movePosSp;
            //            transform.position = newPosition; // We pass it back
            //        }
            //        GUI.Label(new Rect(Screen.width / 2 - 200, 40, 400, 100), step[2], guiStyle);
            //    }
            //    else if (rotCount == 19)
            //    {
            //        Vector3 newPosition = transform.position; // We store the current position
            //        if ((int)transform.position.x != 3 && (int)transform.position.z != 2)
            //        {
            //            newPosition.x = newPosition.x + movePosSp;
            //            newPosition.z = newPosition.z + movePosSp;
            //            transform.position = newPosition; // We pass it back
            //        }
            //        GUI.Label(new Rect(Screen.width / 2 - 200, 40, 400, 100), step[3], guiStyle);
            //    }
            //    else
            //    {
            //        GUI.Label(new Rect(Screen.width / 2 - 200, 40, 400, 100), "Performing", guiStyle);
            //    }

            //    if (rotCount == 2 || rotCount == 11)
            //    {
            //        GetComponent<Renderer>().material.color = Color.green;
            //    }
            //    else if (rotCount == 4 || rotCount == 13)
            //    {
            //        GetComponent<Renderer>().material.color = Color.blue;
            //    }
            //    else if (rotCount == 6 || rotCount == 15)
            //    {
            //        GetComponent<Renderer>().material.color = Color.yellow;
            //    }
            //    else if (rotCount == 8 || rotCount == 17)
            //    {
            //        GetComponent<Renderer>().material.color = Color.red;
            //    }
            //    else
            //    {
            //        GetComponent<Renderer>().material.color = Color.grey;
            //    }
            //}
        }
        // Update is called once per frame
        //void Update(){
        //
        // if (rotCount != 0) {
        // HandOrientation = Quaternion.Slerp (Qref [rotCount - 1], Qref [rotCount], k);
        // HandPosition = Vector3.Lerp (uPref [rotCount - 1], uPref [rotCount], k);
        // if (k < 1) {
        // k += 0.075f;
        // }
        // } else {
        // HandOrientation = Qref [rotCount];
        // HandPosition = uPref [rotCount];
        // }
        //
        // //print ("x=" + HandOrientation.x + " y=" + HandOrientation.y + " z=" + HandOrientation.z + " w=" + HandOrientation.w);
        // this.transform.rotation = HandOrientation;
        //this.transform.position = HandPosition;
        //}
    }
    //ONG single SLERP
    void testOng()
    {
        // Push new data of the measurements into the buffers and calculate the means
        for (int i = BuffSize - 1; i >= 1; i--)
        {
            GyroBuff0[i] = GyroBuff0[i - 1];
            AcceleroBuff0[i] = AcceleroBuff0[i - 1];
            MagnetoBuff0[i] = MagnetoBuff0[i - 1];
            StillnessBuff0[i] = StillnessBuff0[i - 1];
        }

        GyroBuff0[0] = Gyro0;
        AcceleroBuff0[0] = Accelero0;
        MagnetoBuff0[0] = Magneto0;
        StillnessBuff0[0] = Stillness0;

        N += 1;

        int ClampBuffSize = Mathf.Clamp(N, 0, BuffSize);

        GyroAvg0 = new Vector3(0, 0, 0);
        AcceleroAvg0 = new Vector3(0, 0, 0);
        MagnetoAvg0 = new Vector3(0, 0, 0);
        StillnessAvg0 = new float();

        for (int i = 0; i < ClampBuffSize; i++)
        {

            GyroAvg0 += GyroBuff0[i];
            AcceleroAvg0 += AcceleroBuff0[i];
            MagnetoAvg0 += MagnetoBuff0[i];
            StillnessAvg0 += StillnessBuff0[i];
        }

        GyroAvg0 /= ClampBuffSize;
        AcceleroAvg0 /= ClampBuffSize;
        MagnetoAvg0 /= ClampBuffSize;
        thisAlphaX0 = Mathf.Pow((StillnessAvg0 / ClampBuffSize), 2.0f);
        thisAlphaY0 = alpWeight * (prevAlphaX0) + (1.0f - alpWeight) * (prevAlphaY0); //Gamma memory filter
        prevAlphaX0 = thisAlphaX0;
        prevAlphaY0 = thisAlphaY0;

        // Calculate new Bias offset Errors when the sensor is NOT rotating

        if (Mathf.Abs(Gyro0.x) < 0.03 && Mathf.Abs(Gyro0.y) < 0.03 && Mathf.Abs(Gyro0.z) < 0.03)
        {
            BiasBuff0.x += Gyro0.x;
            BiasBuff0.y += Gyro0.y;
            BiasBuff0.z += Gyro0.z;
            trigcount0 += 1;

        }
        else
        {
            trigcount0 = 0;
            BiasBuff0.x = 0;
            BiasBuff0.y = 0;
            BiasBuff0.z = 0;
        }

        if (trigcount0 == 5)
        {

            Bias0 = new Vector3(BiasBuff0.x / 5.0f, BiasBuff0.y / 5.0f, BiasBuff0.z / 5.0f);
            trigcount0 = 0;
            BiasBuff0.x = 0;
            BiasBuff0.y = 0;
            BiasBuff0.z = 0;
        }

        // Remove Gyroscope Bias

        UnbiasedGyro0 = Gyro0 - (B_SCALING * Bias0);
        UnbiasedGyroWithfixedBias0 = Gyro0 - fixedBias0;

        // Compute Quaternions

        if (N == 1)
        {
            A_int0 = new Quaternion(Accelero0.x, Accelero0.y, Accelero0.z, 0.0f);
            M_int0 = new Quaternion(Magneto0.x, Magneto0.y, Magneto0.z, 0.0f);
        }

        wfixed0 = new Quaternion(UnbiasedGyroWithfixedBias0.x, UnbiasedGyroWithfixedBias0.y, UnbiasedGyroWithfixedBias0.z, 0.0f);
        w0 = new Quaternion(UnbiasedGyro0.x, UnbiasedGyro0.y, UnbiasedGyro0.z, 0.0f);

        dqGfixed0 = (qGfixed0 * wfixed0);
        dqGfixed0 = new Quaternion(0.5f * dqGfixed0.x, 0.5f * dqGfixed0.y, 0.5f * dqGfixed0.z, 0.5f * dqGfixed0.w);
        qGfixed0 = myQuatIntegrate(dqGfixed0, qGfixed0, SamplingTime);
        qGfixed0 = myQuatNormalize(qGfixed0);

        qG0 = qOUT0;
        qGA0 = qOUT0;
        qGM0 = qOUT0;

        dqG0 = (qG0 * w0);
        dqG0 = new Quaternion(0.5f * dqG0.x, 0.5f * dqG0.y, 0.5f * dqG0.z, 0.5f * dqG0.w);
        qG0 = myQuatIntegrate(dqG0, qG0, SamplingTime);
        qG0 = myQuatNormalize(qG0);

        dqGA0 = (qGA0 * w0);
        dqGA0 = new Quaternion(0.5f * dqGA0.x, 0.5f * dqGA0.y, 0.5f * dqGA0.z, 0.5f * dqGA0.w);
        qGA0 = myQuatIntegrate(dqGA0, qGA0, SamplingTime);
        qGA0 = myQuatNormalize(qGA0);

        dqGM0 = (qGM0 * w0);
        dqGM0 = new Quaternion(0.5f * dqGM0.x, 0.5f * dqGM0.y, 0.5f * dqGM0.z, 0.5f * dqGM0.w);
        qGM0 = myQuatIntegrate(dqGM0, qGM0, SamplingTime);
        qGM0 = myQuatNormalize(qGM0);

        // Compute Gravity and Magnetic North Vectors

        a40 = myQuatConj(qGA0) * (A_int0 * qGA0);
        a30 = new Vector3(a40.x, a40.y, a40.z);
        m40 = myQuatConj(qGM0) * (M_int0 * qGM0);
        m30 = new Vector3(m40.x, m40.y, m40.z);

        // Compute Differences between measured and calculated Gravity Vector described in Quaternion domain

        Vector3 qAv0 = Vector3.Cross(AcceleroAvg0, a30);
        float qAw0 = Vector3.Magnitude(AcceleroAvg0) * Vector3.Magnitude(a30) + Vector3.Dot(AcceleroAvg0, a30);
        Quaternion deltaQa0 = myQuatNormalize(new Quaternion(qAv0.x, qAv0.y, qAv0.z, qAw0));
        qGA0 = myQuatNormalize(qGA0 * deltaQa0);

        // Compute Differences between measured and calculated Magnetic North Vector described in Quaternion domain

        Vector3 qMv0 = Vector3.Cross(Magneto0, m30);
        float qMw0 = Vector3.Magnitude(Magneto0) * Vector3.Magnitude(m30) + Vector3.Dot(Magneto0, m30);
        Quaternion deltaQm0 = myQuatNormalize(new Quaternion(qMv0.x, qMv0.y, qMv0.z, qMw0));
        qGM0 = myQuatNormalize(qGM0 * deltaQm0);

        // Quaternion Interpolation

        alpha0 = thisAlphaY0;

        alpha0 = (ma * alpha0) + (1 - ma); //Linear Equation
        alpha0 = (alpha0 + (Math.Abs(alpha0))) / 2;
        if (alpha0 < 0.01f)
        {
            alpha0 = 0.5f;
        }
        else
        {
            alpha0 = alpha0;
        }

        //SLERP
        qOUT0 = Quaternion.Slerp(qGM0, qGA0, alpha0);

        //Conversion of Angles from Quaternion to Euler Angles

        //EulerFixed0 = qGfixed0.eulerAngles;
        //EulerKalman0 = IMUQuat0.eulerAngles;
        //EulerGMV0 = qOUT0.eulerAngles;


        // if (EulerFixed0.x > 180.0)
        // EulerFixed0.x = -(360.0f - EulerFixed0.x);
        // if (EulerFixed0.y > 180.0)
        // EulerFixed0.y = -(360.0f - EulerFixed0.y);
        // if (EulerFixed0.z > 180.0)
        // EulerFixed0.z = -(360.0f - EulerFixed0.z);
        //
        // if (EulerKalman0.x > 180.0)
        // EulerKalman0.x = -(360.0f - EulerKalman0.x);
        // if (EulerKalman0.y > 180.0)
        // EulerKalman0.y = -(360.0f - EulerKalman0.y);
        // if (EulerKalman0.z > 180.0)
        // EulerKalman0.z = -(360.0f - EulerKalman0.z);
        //
        // if (EulerGMV0.x > 180.0)
        // EulerGMV0.x = -(360.0f - EulerGMV0.x);
        // if (EulerGMV0.y > 180.0)
        // EulerGMV0.y = -(360.0f - EulerGMV0.y);
        // if (EulerGMV0.z > 180.0)
        // EulerGMV0.z = -(360.0f - EulerGMV0.z);

        // Print the data to a file
        long milliseconds = DateTimeOffset.Now.ToUnixTimeMilliseconds();
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("T{0},{1},{2},{3},{4}", milliseconds, qGfixed0.x, qGfixed0.y, qGfixed0.z, qGfixed0.w));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},{1},{2},{3},", IMUQuat0.x, IMUQuat0.y, IMUQuat0.z, IMUQuat0.w));
        //System.IO.File.AppendAllText(FileStr0, System.String.Foat("{0},{1},{2},{3},", qOUT0.x, qOUT0.y, qOUT0.z, qOUT0.w));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},{1},{2},", UnbiasedGyro0.x, UnbiasedGyro0.y, UnbiasedGyro0.z));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},{1},{2},", Bias0.x, Bias0.y, Bias0.z));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},", alpha0));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},{1},{2},{3},", qGA0.x, qGA0.y, qGA0.z, qGA0.w));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},{1},{2},{3},", qGM0.x, qGM0.y, qGM0.z, qGM0.w));
        //System.IO.File.AppendAllText(FileStr0, System.String.Format("{0},{1},{2}\n", Gyro0.x, Gyro0.y, Gyro0.z));

        //System.IO.File.AppendAllText(FileStr0, System.String.Format("T{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15} \n"
        //    , CurrentTime, StillnessAvg0, Gyro0.x, Gyro0.y, Gyro0.z, Accelero0.x, Accelero0.y, Accelero0.z, Magneto0.x, Magneto0.y, Magneto0.z, alpha0, qOUT0.x, qOUT0.y, qOUT0.z, qOUT0.w));
    }

    void testNan()
    {
        exectimeStart.Add(tu.GetNanoseconds());
        for (int i = BuffSize - 1; i >= 1; i--)
        {
            GyroBuff0[i] = GyroBuff0[i - 1];
            AcceleroBuff0[i] = AcceleroBuff0[i - 1];
            MagnetoBuff0[i] = MagnetoBuff0[i - 1];
            StillnessBuff0[i] = StillnessBuff0[i - 1];
        }

        GyroBuff0[0] = Gyro0;
        AcceleroBuff0[0] = Accelero0;
        MagnetoBuff0[0] = Magneto0;
        StillnessBuff0[0] = Stillness0;

        N += 1;

        int ClampBuffSize = Mathf.Clamp(N, 0, BuffSize);

        GyroAvg0 = new Vector3(0, 0, 0);
        AcceleroAvg0 = new Vector3(0, 0, 0);
        MagnetoAvg0 = new Vector3(0, 0, 0);
        StillnessAvg0 = new float();

        for (int i = 0; i < ClampBuffSize; i++)
        {

            GyroAvg0 += GyroBuff0[i];
            AcceleroAvg0 += AcceleroBuff0[i];
            MagnetoAvg0 += MagnetoBuff0[i];
            StillnessAvg0 += StillnessBuff0[i];
        }

        GyroAvg0 /= ClampBuffSize;
        AcceleroAvg0 /= ClampBuffSize;
        MagnetoAvg0 /= ClampBuffSize;
        thisAlphaX0 = Mathf.Pow((StillnessAvg0 / ClampBuffSize), 2.0f);
        thisAlphaY0 = alpWeight * (prevAlphaX0) + (1.0f - alpWeight) * (prevAlphaY0);
        prevAlphaX0 = thisAlphaX0;
        prevAlphaY0 = thisAlphaY0;

        // Calculate new Bias offset Errors when the sensor is NOT rotating

        if (Mathf.Abs(Gyro0.x) < 0.03 && Mathf.Abs(Gyro0.y) < 0.03 && Mathf.Abs(Gyro0.z) < 0.03)
        {
            BiasBuff0.x += Gyro0.x;
            BiasBuff0.y += Gyro0.y;
            BiasBuff0.z += Gyro0.z;
            trigcount0 += 1;

        }
        else
        {
            trigcount0 = 0;
            BiasBuff0.x = 0;
            BiasBuff0.y = 0;
            BiasBuff0.z = 0;
        }

        if (trigcount0 == 5)
        {

            Bias0 = new Vector3(BiasBuff0.x / 5.0f, BiasBuff0.y / 5.0f, BiasBuff0.z / 5.0f);
            trigcount0 = 0;
            BiasBuff0.x = 0;
            BiasBuff0.y = 0;
            BiasBuff0.z = 0;
        }

        // Remove Gyroscope Bias

        UnbiasedGyro0 = Gyro0 - (B_SCALING * Bias0);

        // Compute Quaternions

        if (N == 1)
        {
            A_int0 = new Quaternion(Accelero0.x, Accelero0.y, Accelero0.z, 0.0f);
            M_int0 = new Quaternion(Magneto0.x, Magneto0.y, Magneto0.z, 0.0f);
        }

        w0 = new Quaternion(UnbiasedGyro0.x, UnbiasedGyro0.y, UnbiasedGyro0.z, 0.0f);


        qG0 = qOUT1;
        qGA0 = qOUT1;
        qGM0 = qOUT1;

        dqG0 = (qG0 * w0);
        dqG0 = new Quaternion(0.5f * dqG0.x, 0.5f * dqG0.y, 0.5f * dqG0.z, 0.5f * dqG0.w);
        qG0 = myQuatIntegrate(dqG0, qG0, SamplingTime);
        qG0 = myQuatNormalize(qG0);

        dqGA0 = (qGA0 * w0);
        dqGA0 = new Quaternion(0.5f * dqGA0.x, 0.5f * dqGA0.y, 0.5f * dqGA0.z, 0.5f * dqGA0.w);
        qGA0 = myQuatIntegrate(dqGA0, qGA0, SamplingTime);
        qGA0 = myQuatNormalize(qGA0);

        dqGM0 = (qGM0 * w0);
        dqGM0 = new Quaternion(0.5f * dqGM0.x, 0.5f * dqGM0.y, 0.5f * dqGM0.z, 0.5f * dqGM0.w);
        qGM0 = myQuatIntegrate(dqGM0, qGM0, SamplingTime);
        qGM0 = myQuatNormalize(qGM0);

        // Compute Gravity and Magnetic North Vectors

        a40 = myQuatConj(qGA0) * (A_int0 * qGA0);
        a30 = new Vector3(a40.x, a40.y, a40.z);
        m40 = myQuatConj(qGM0) * (M_int0 * qGM0);
        m30 = new Vector3(m40.x, m40.y, m40.z);

        // Compute Differences between measured and calculated Gravity Vector described in Quaternion domain

        Vector3 qAv0 = Vector3.Cross(AcceleroAvg0, a30);
        float qAw0 = Vector3.Magnitude(AcceleroAvg0) * Vector3.Magnitude(a30) + Vector3.Dot(AcceleroAvg0, a30);
        Quaternion deltaQa0 = myQuatNormalize(new Quaternion(qAv0.x, qAv0.y, qAv0.z, qAw0));
        qGA0 = myQuatNormalize(qGA0 * deltaQa0);


        // Compute Differences between measured and calculated Magnetic North Vector described in Quaternion domain

        Vector3 qMv0 = Vector3.Cross(Magneto0, m30);
        float qMw0 = Vector3.Magnitude(Magneto0) * Vector3.Magnitude(m30) + Vector3.Dot(Magneto0, m30);
        Quaternion deltaQm0 = myQuatNormalize(new Quaternion(qMv0.x, qMv0.y, qMv0.z, qMw0));
        qGM0 = myQuatNormalize(qGM0 * deltaQm0);

        // Quaternion Interpolation

        alpha0 = thisAlphaY0;

        //////////////////////////////////////////////
        ///GENERATE (alpha)
        //////////////////////////////////////////////

        alpha0 = (ma * alpha0) + (1 - ma); //Linear Equation
        alpha0 = (alpha0 + (Math.Abs(alpha0))) / 2;
        if (alpha0 < 0.01f)
        {
            alpha0 = 0.5f;
        }
        else
        {
            alpha0 = alpha0;
        }


        //////////////////////////////////////////////
        ///GENERATE (MU)
        //////////////////////////////////////////////

        qGpost = qGA0;
        mGA4 = myQuatConj(qGpost) * (M_int0 * qGpost);
        mGA3 = new Vector3(mGA4.x, mGA4.y, mGA4.z);

        ////Vector3 testMagnet = new Vector3((int)MagnetoAvg0.x * 100, (int)MagnetoAvg0.y * 100, (int)MagnetoAvg0.z * 100);
        ////Vector3 testmGA3 = new Vector3((int)mGA3.x * 100, (int)mGA3.y * 100, (int)mGA3.z * 100);
        ////float Gamma = Vector3.Angle(testMagnet, testmGA3);
        //float Gamma = Vector3.Angle(MagnetoAvg0, mGA3);
        //Debug.Log(testMagnet + "***" + testmGA3 + "***" + Gamma);

        //System.IO.File.AppendAllText(FileStr0, System.String.Format("T{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10} \n"
        //    , CurrentTime, StillnessAvg0, Gyro0.x, Gyro0.y, Gyro0.z, Accelero0.x, Accelero0.y, Accelero0.z, Magneto0.x, Magneto0.y, Magneto0.z));
        norm_MagAvg = (float)Math.Sqrt((Magneto0.x * Magneto0.x) + (Magneto0.y * Magneto0.y) + (Magneto0.z * Magneto0.z));
        norm_mGA3 = (float)Math.Sqrt((mGA3.x * mGA3.x) + (mGA3.y * mGA3.y) + (mGA3.z * mGA3.z));
        cosGamma = Vector3.Dot(MagnetoAvg0, mGA3);
        cosGamma = cosGamma / (norm_MagAvg * norm_mGA3);
        if (cosGamma > 1.0f)
            cosGamma = 1.0f;
        if (cosGamma < -1.0f)
            cosGamma = -1.0f;

        ////cosGamma = MagnetoAvg0.dot(mGA3);
        gg = (float)Math.Acos((double)cosGamma);

        //gg = Gamma;
        gl = -mt * gg + 1.0f; //Linear Equation
        temp_mu = (1.0f + gl) / 2.0f;
        //temp_mu = gg;
        if (temp_mu < 0)
        {
            thisMu = 0;
        }
        else
        {
            thisMu = temp_mu;
        }

        float thisMuY = 0.0f;
        float thisMuX = thisMu * thisMu;

        thisMuY = (muWeight * prevMuX) + (1.0f - muWeight) * prevMuY;
        prevMuX = thisMuX;
        prevMuY = thisMuY;
        // Put gramma here.
        /*
         *  thisAlphaX0 = Mathf.Pow((StillnessAvg0 / ClampBuffSize), 2.0f);
            thisAlphaY0 = alpWeight * (prevAlphaX0) + (1.0f - alpWeight) * (prevAlphaY0);
            prevAlphaX0 = thisAlphaX0;
            prevAlphaY0 = thisAlphaY0;
         */
        // Modified by PS 4/13/2022
        //LocateVOX(OptitrackStreamingClient.markerX, OptitrackStreamingClient.markerY, OptitrackStreamingClient.markerZ);

        if (MU[LocateVoxX, LocateVoxY, LocateVoxZ] < TM && alpha0 > TA)
        {
            MU[LocateVoxX, LocateVoxY, LocateVoxZ] = thisMuY;
        }
        qOUT1 = Quaternion.Slerp((Quaternion.Slerp(qG0, qGM0, MU[LocateVoxX, LocateVoxY, LocateVoxZ])), (Quaternion.Slerp(qG0, qGA0, alpha0)), alpha0);
        // qOUT1 = Quaternion.Slerp((Quaternion.Slerp(qG0, qGM0, 0)), (Quaternion.Slerp(qG0, qGA0, alpha0)), alpha0);


        Debug.Log(qOUT1);


        // Debug.Log(MU[LocateVoxX, LocateVoxY, LocateVoxZ] );
        //qOUT1 = Quaternion.Slerp(qG0, qGM0, mu0);

        //Conversion of Angles from Quaternion to Euler Angles

        //EulerFixed0 = qGfixed0.eulerAngles;
        //EulerKalman0 = IMUQuat0.eulerAngles;
        //EulerGMV1 = qOUT1.eulerAngles;

        // if (EulerFixed0.x > 180.0)
        // EulerFixed0.x = -(360.0f - EulerFixed0.x);
        // if (EulerFixed0.y > 180.0)
        // EulerFixed0.y = -(360.0f - EulerFixed0.y);
        // if (EulerFixed0.z > 180.0)
        // EulerFixed0.z = -(360.0f - EulerFixed0.z);
        //
        // if (EulerKalman0.x > 180.0)
        // EulerKalman0.x = -(360.0f - EulerKalman0.x);
        // if (EulerKalman0.y > 180.0)
        // EulerKalman0.y = -(360.0f - EulerKalman0.y);
        // if (EulerKalman0.z > 180.0)
        // EulerKalman0.z = -(360.0f - EulerKalman0.z);
        //
        // if (EulerGMV0.x > 180.0)
        // EulerGMV0.x = -(360.0f - EulerGMV0.x);
        // if (EulerGMV0.y > 180.0)
        // EulerGMV0.y = -(360.0f - EulerGMV0.y);
        // if (EulerGMV0.z > 180.0)
        // EulerGMV0.z = -(360.0f - EulerGMV0.z);

        // Print the data to a file
        //System.IO.File.AppendAllText(FileStr1, System.String.Format("T{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19} \n"
        //    , CurrentTime, StillnessAvg0, Gyro0.x, Gyro0.y, Gyro0.z, Accelero0.x, Accelero0.y, Accelero0.z, Magneto0.x, Magneto0.y, Magneto0.z, alpha0, qOUT1.x, qOUT1.y, qOUT1.z, qOUT1.w, LocateVoxX, LocateVoxY, LocateVoxZ, MU[LocateVoxX, LocateVoxY, LocateVoxZ]));
        long milliseconds = DateTimeOffset.Now.ToUnixTimeMilliseconds();
        exectimeStop.Add(tu.GetNanoseconds());
    }

    private static void LocateVOX(float x, float y, float z)
    {
        float OffsetNewOriginX = (float)-50.0; //Set New origin at x: -200
        float OffsetNewOriginY = (float)-50.0; //Set New origin at y: -200

        float MPx = (x);
        float MPy = (y);
        float MPz = (z);

        float nMPx = MPx - OffsetNewOriginX;
        float nMPy = MPy - OffsetNewOriginY;
        float nMPz = MPz;

        int VoxelSize = 1;

        LocateVoxX = (int)(nMPx / VoxelSize) + 1;
        LocateVoxY = (int)(nMPy / VoxelSize) + 1;
        LocateVoxZ = (int)(nMPz / VoxelSize) + 1;
    }
}