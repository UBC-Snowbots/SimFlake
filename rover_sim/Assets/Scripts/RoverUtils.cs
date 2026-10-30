using SimFlake.Ros;

/// <summary>Kept for older scripts. New code can use SimFlake.Ros.RosTime directly.</summary>
public static class RoverUtils
{
    /// <summary>Current wall-clock time as a ROS timestamp.</summary>
    public static builtin_interfaces.msg.Time GetROS2Timestamp() => RosTime.Now();

    /// <summary>A Header with the given frame id, stamped now.</summary>
    public static std_msgs.msg.Header CreateHeader(string frameId) => RosTime.Header(frameId);
}
