using System;
using builtin_interfaces.msg;
using std_msgs.msg;

public static class RoverUtils
{
    // Epoch time (Jan 1, 1970)
    private static readonly DateTime Epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    /// <summary>
    /// Generates a ROS2 timestamp (builtin_interfaces.msg.Time) from the current UTC time.
    /// </summary>
    public static Time GetROS2Timestamp()
    {
        TimeSpan timeSinceEpoch = DateTime.UtcNow - Epoch;
        double totalSeconds = timeSinceEpoch.TotalSeconds;
        int sec = (int)totalSeconds; // Integer seconds
        uint nanosec = (uint)((totalSeconds - sec) * 1e9); // Fractional part to nanoseconds

        return new Time { Sec = sec, Nanosec = nanosec };
    }

    /// <summary>
    /// Creates a ROS2 Header with a specified frame ID and current timestamp, off of system time.
    /// </summary>
    public static Header CreateHeader(string frameId)
    {
        return new Header
        {
            Frame_id = frameId,
            Stamp = GetROS2Timestamp()
        };
    }
}