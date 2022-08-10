using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

public class TimeUtils
{
	public long GetNanoseconds()
	{
		double timestamp = Stopwatch.GetTimestamp();
		double nanoseconds = 1_000_000_000.0 * timestamp / Stopwatch.Frequency;

		return (long)nanoseconds;
	}
}
