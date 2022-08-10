using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

using System.IO.Ports;

public class SensorConnection : MonoBehaviour
{
    // Connect to the serial port the 3-Space Sensor is connected to
    private static SerialPort sp0 = new SerialPort("\\\\.\\COM3", 115200, Parity.None, 8, StopBits.One);
    private static byte TSS_START_BYTE = 0xF7;
    private static byte TSS_SET_STREAMING_SLOTS = 0x50;
    private static byte TSS_SET_STREAMING_TIMING = 0x52;
    private static byte TSS_START_STREAMING = 0x55;
    private static byte TSS_STOP_STREAMING = 0x56;
    private static byte TSS_GET_SENSOR_MOTION = 0x2D;
    private static byte TSS_GET_RAD_PER_SEC_GYROSCOPE = 0x26;
    private static byte TSS_GET_CORRETED_LINEAR_ACC_AND_GRAVITY = 0x27;
    private static byte TSS_GET_CORRETED_COMPASS = 0x28;
    private static byte TSS_GET_TARED_ORIENTAITON_AS_QUAT = 0x00;
    private static byte TSS_NULL = 0xFF;
    private static byte TSS_TARE_CURRENT_ORIENTATION = 0x60;
    private static byte CHECK_SUM = (byte)((TSS_SET_STREAMING_SLOTS + TSS_GET_SENSOR_MOTION + TSS_GET_RAD_PER_SEC_GYROSCOPE + TSS_GET_CORRETED_LINEAR_ACC_AND_GRAVITY +
        TSS_GET_CORRETED_COMPASS + TSS_GET_TARED_ORIENTAITON_AS_QUAT + TSS_NULL + TSS_NULL + TSS_NULL) % 256);

    private static byte[] stream_slots_bytes = {TSS_START_BYTE,
                                        TSS_SET_STREAMING_SLOTS,
                                        TSS_GET_SENSOR_MOTION, // Slot0 - 4
                                        TSS_GET_RAD_PER_SEC_GYROSCOPE, // Slot1 - 12
                                        TSS_GET_CORRETED_LINEAR_ACC_AND_GRAVITY, // Slot2 - 12
                                        TSS_GET_CORRETED_COMPASS, // Slot3 - 12
                                        TSS_GET_TARED_ORIENTAITON_AS_QUAT, // Slot4 - 16
                                        TSS_NULL, // Slot5
                                        TSS_NULL, // Slot6
                                        TSS_NULL, // Slot7
                                        CHECK_SUM};


    private static byte[] stream_timing_bytes = new byte[15];
    private static byte[] start_stream_bytes = new byte[3];
    private static byte[] tare_bytes = { TSS_START_BYTE, TSS_TARE_CURRENT_ORIENTATION, TSS_TARE_CURRENT_ORIENTATION }; // <-- one command, checksum = command itself
    private static byte[] interval = BitConverter.GetBytes(70000);
    private static byte[] delay = BitConverter.GetBytes(0);
    private static byte[] duration = BitConverter.GetBytes(0xFFFFFFFF);

    public SerialPort ConnectionInit() {
        
        sp0.WriteTimeout = 500;
        sp0.ReadTimeout = 500;

                if (!sp0.IsOpen)
        {
            try
            {
                sp0.Open();

                print("Serial Port 0 is open (COM4)");
            }
            catch (TimeoutException)
            {
                print("Serial Port 0 is open (COM4)");
            }

        }
        else
        {
            Debug.LogError("All Serial Ports are already open.");
            print("Serial Port 0 is open (COM4)");
        }


        Array.Reverse(interval);
        Array.Reverse(delay);
        Array.Reverse(duration);

        stream_timing_bytes[0] = TSS_START_BYTE;
        stream_timing_bytes[1] = TSS_SET_STREAMING_TIMING;
        interval.CopyTo(stream_timing_bytes, 2);
        delay.CopyTo(stream_timing_bytes, 6);
        duration.CopyTo(stream_timing_bytes, 10);
        stream_timing_bytes[14] = (byte)((stream_timing_bytes[1] + stream_timing_bytes[2] + stream_timing_bytes[3] + stream_timing_bytes[4] + stream_timing_bytes[5] + stream_timing_bytes[6] +
        stream_timing_bytes[7] + stream_timing_bytes[8] + stream_timing_bytes[9] + stream_timing_bytes[10] + stream_timing_bytes[11] + stream_timing_bytes[12] + stream_timing_bytes[13]) % 256);

        sp0.Write(stream_timing_bytes, 0, stream_timing_bytes.Length);
        sp0.Write(stream_slots_bytes, 0, stream_slots_bytes.Length);

        sp0.Write(tare_bytes, 0, 3); // tareSensor

        start_stream_bytes[0] = TSS_START_BYTE;
        start_stream_bytes[1] = TSS_START_STREAMING;
        start_stream_bytes[2] = TSS_START_STREAMING;

        sp0.Write(start_stream_bytes, 0, start_stream_bytes.Length);
        return sp0;
    }
}
