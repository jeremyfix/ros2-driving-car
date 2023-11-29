using System;
using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

namespace RosUtilities.MsgHandling
{
    public class Utils
    {
        public const double k_NanosecondsInSecond = 1e9f;

        public static HeaderMsg StampedHeader(string frameID)
        {
            // getTime of last FixedUpdate
            var timeInSeconds = Time.fixedTimeAsDouble;
            var sec = Math.Floor(timeInSeconds);
            var nano = (timeInSeconds - sec) * k_NanosecondsInSecond;


            return new HeaderMsg(
                new TimeMsg((int)sec, (uint)nano),
                frameID
            );
        }
    }
}