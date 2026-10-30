// ROS 2 message definitions used by the sim.
//
// These are plain [Serializable] C# classes whose field names match the ROS 2
// .msg field names exactly. Unity's built-in JsonUtility turns them into the
// JSON that rosbridge expects, so no code generation or native libraries are
// needed.
//
// Adding a new message type:
//   1. Make a [Serializable] class deriving from RosMessage.
//   2. Add [RosMsg("pkg/msg/Type")].
//   3. Add public fields with the same names/types as the .msg file.
//      Fixed-size arrays (e.g. float64[9]) must be created at the right size.
//      uint8[] fields: see Image / CompressedImage (IBinaryMessage).
// Custom RoverFlake2 messages (rover_msgs/...) work too, as long as rosbridge
// was started with RoverFlake2 sourced.

using System;
using SimFlake.Ros;

namespace builtin_interfaces.msg
{
    [Serializable, RosMsg("builtin_interfaces/msg/Time")]
    public class Time : RosMessage
    {
        public int sec;
        public uint nanosec;
    }
}

namespace std_msgs.msg
{
    using builtin_interfaces.msg;

    [Serializable, RosMsg("std_msgs/msg/Header")]
    public class Header : RosMessage
    {
        public Time stamp = new Time();
        public string frame_id = "";
    }

    [Serializable, RosMsg("std_msgs/msg/String")]
    public class String : RosMessage
    {
        public string data = "";
    }

    [Serializable, RosMsg("std_msgs/msg/Bool")]
    public class Bool : RosMessage
    {
        public bool data;
    }

    [Serializable, RosMsg("std_msgs/msg/Float32")]
    public class Float32 : RosMessage
    {
        public float data;
    }

    [Serializable, RosMsg("std_msgs/msg/Float64")]
    public class Float64 : RosMessage
    {
        public double data;
    }

    [Serializable, RosMsg("std_msgs/msg/Int32")]
    public class Int32 : RosMessage
    {
        public int data;
    }

    [Serializable, RosMsg("std_msgs/msg/MultiArrayDimension")]
    public class MultiArrayDimension : RosMessage
    {
        public string label = "";
        public uint size;
        public uint stride;
    }

    [Serializable, RosMsg("std_msgs/msg/MultiArrayLayout")]
    public class MultiArrayLayout : RosMessage
    {
        public MultiArrayDimension[] dim = new MultiArrayDimension[0];
        public uint data_offset;
    }

    [Serializable, RosMsg("std_msgs/msg/Float64MultiArray")]
    public class Float64MultiArray : RosMessage
    {
        public MultiArrayLayout layout = new MultiArrayLayout();
        public double[] data = new double[0];
    }

    [Serializable, RosMsg("std_msgs/msg/Float32MultiArray")]
    public class Float32MultiArray : RosMessage
    {
        public MultiArrayLayout layout = new MultiArrayLayout();
        public float[] data = new float[0];
    }
}

namespace geometry_msgs.msg
{
    using std_msgs.msg;

    [Serializable, RosMsg("geometry_msgs/msg/Vector3")]
    public class Vector3 : RosMessage
    {
        public double x, y, z;
        public Vector3() { }
        public Vector3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    [Serializable, RosMsg("geometry_msgs/msg/Point")]
    public class Point : RosMessage
    {
        public double x, y, z;
        public Point() { }
        public Point(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    }

    [Serializable, RosMsg("geometry_msgs/msg/Quaternion")]
    public class Quaternion : RosMessage
    {
        public double x, y, z;
        public double w = 1;
        public Quaternion() { }
        public Quaternion(double x, double y, double z, double w) { this.x = x; this.y = y; this.z = z; this.w = w; }
    }

    [Serializable, RosMsg("geometry_msgs/msg/Pose")]
    public class Pose : RosMessage
    {
        public Point position = new Point();
        public Quaternion orientation = new Quaternion();
    }

    [Serializable, RosMsg("geometry_msgs/msg/PoseWithCovariance")]
    public class PoseWithCovariance : RosMessage
    {
        public Pose pose = new Pose();
        public double[] covariance = new double[36];
    }

    [Serializable, RosMsg("geometry_msgs/msg/Twist")]
    public class Twist : RosMessage
    {
        public Vector3 linear = new Vector3();
        public Vector3 angular = new Vector3();
    }

    [Serializable, RosMsg("geometry_msgs/msg/TwistWithCovariance")]
    public class TwistWithCovariance : RosMessage
    {
        public Twist twist = new Twist();
        public double[] covariance = new double[36];
    }

    [Serializable, RosMsg("geometry_msgs/msg/Transform")]
    public class Transform : RosMessage
    {
        public Vector3 translation = new Vector3();
        public Quaternion rotation = new Quaternion();
    }

    [Serializable, RosMsg("geometry_msgs/msg/TransformStamped")]
    public class TransformStamped : RosMessage
    {
        public Header header = new Header();
        public string child_frame_id = "";
        public Transform transform = new Transform();
    }
}

namespace sensor_msgs.msg
{
    using std_msgs.msg;
    using geometry_msgs.msg;

    [Serializable, RosMsg("sensor_msgs/msg/Imu")]
    public class Imu : RosMessage
    {
        public Header header = new Header();
        public Quaternion orientation = new Quaternion();
        public double[] orientation_covariance = new double[9];
        public Vector3 angular_velocity = new Vector3();
        public double[] angular_velocity_covariance = new double[9];
        public Vector3 linear_acceleration = new Vector3();
        public double[] linear_acceleration_covariance = new double[9];
    }

    [Serializable, RosMsg("sensor_msgs/msg/NavSatStatus")]
    public class NavSatStatus : RosMessage
    {
        public const sbyte STATUS_NO_FIX = -1, STATUS_FIX = 0, STATUS_SBAS_FIX = 1, STATUS_GBAS_FIX = 2;
        public const ushort SERVICE_GPS = 1, SERVICE_GLONASS = 2, SERVICE_COMPASS = 4, SERVICE_GALILEO = 8;
        public sbyte status;
        public ushort service = SERVICE_GPS;
    }

    [Serializable, RosMsg("sensor_msgs/msg/NavSatFix")]
    public class NavSatFix : RosMessage
    {
        public const byte COVARIANCE_TYPE_UNKNOWN = 0, COVARIANCE_TYPE_APPROXIMATED = 1,
                          COVARIANCE_TYPE_DIAGONAL_KNOWN = 2, COVARIANCE_TYPE_KNOWN = 3;
        public Header header = new Header();
        public NavSatStatus status = new NavSatStatus();
        public double latitude;
        public double longitude;
        public double altitude;
        public double[] position_covariance = new double[9];
        public byte position_covariance_type;
    }

    [Serializable, RosMsg("sensor_msgs/msg/LaserScan")]
    public class LaserScan : RosMessage
    {
        public Header header = new Header();
        public float angle_min, angle_max, angle_increment;
        public float time_increment, scan_time;
        public float range_min, range_max;
        public float[] ranges = new float[0];
        public float[] intensities = new float[0];
    }

    [Serializable, RosMsg("sensor_msgs/msg/JointState")]
    public class JointState : RosMessage
    {
        public Header header = new Header();
        public string[] name = new string[0];
        public double[] position = new double[0];
        public double[] velocity = new double[0];
        public double[] effort = new double[0];
    }

    /// <summary>Raw image. Pixel bytes go in <see cref="data"/> (sent as base64, not via JsonUtility).</summary>
    [Serializable, RosMsg("sensor_msgs/msg/Image")]
    public class Image : RosMessage, IBinaryMessage
    {
        public Header header = new Header();
        public uint height;
        public uint width;
        public string encoding = "rgb8";
        public byte is_bigendian;
        public uint step;
        [NonSerialized] public byte[] data;
        /// <summary>Number of valid bytes in <see cref="data"/>; -1 means the whole array (lets you use pooled buffers).</summary>
        [NonSerialized] public int dataLength = -1;

        public string BinaryFieldName => "data";
        public ArraySegment<byte> BinaryData =>
            data == null ? new ArraySegment<byte>(Array.Empty<byte>())
                         : new ArraySegment<byte>(data, 0, dataLength < 0 ? data.Length : dataLength);
    }

    [Serializable, RosMsg("sensor_msgs/msg/CompressedImage")]
    public class CompressedImage : RosMessage, IBinaryMessage
    {
        public Header header = new Header();
        public string format = "jpeg";
        [NonSerialized] public byte[] data;
        [NonSerialized] public int dataLength = -1;

        public string BinaryFieldName => "data";
        public ArraySegment<byte> BinaryData =>
            data == null ? new ArraySegment<byte>(Array.Empty<byte>())
                         : new ArraySegment<byte>(data, 0, dataLength < 0 ? data.Length : dataLength);
    }

    [Serializable, RosMsg("sensor_msgs/msg/RegionOfInterest")]
    public class RegionOfInterest : RosMessage
    {
        public uint x_offset, y_offset, height, width;
        public bool do_rectify;
    }

    [Serializable, RosMsg("sensor_msgs/msg/CameraInfo")]
    public class CameraInfo : RosMessage
    {
        public Header header = new Header();
        public uint height;
        public uint width;
        public string distortion_model = "plumb_bob";
        public double[] d = new double[5];
        public double[] k = new double[9];
        public double[] r = new double[9];
        public double[] p = new double[12];
        public uint binning_x;
        public uint binning_y;
        public RegionOfInterest roi = new RegionOfInterest();
    }
}

namespace nav_msgs.msg
{
    using std_msgs.msg;
    using geometry_msgs.msg;

    [Serializable, RosMsg("nav_msgs/msg/Odometry")]
    public class Odometry : RosMessage
    {
        public Header header = new Header();
        public string child_frame_id = "";
        public PoseWithCovariance pose = new PoseWithCovariance();
        public TwistWithCovariance twist = new TwistWithCovariance();
    }
}

namespace tf2_msgs.msg
{
    using geometry_msgs.msg;

    [Serializable, RosMsg("tf2_msgs/msg/TFMessage")]
    public class TFMessage : RosMessage
    {
        public TransformStamped[] transforms = new TransformStamped[0];
    }
}

// RoverFlake2 custom messages. rosbridge needs RoverFlake2 sourced to use these
// (it is when it starts through ~/.bashrc / ~/.roverrc).
namespace rover_msgs.msg
{
    /// <summary>rover_msgs/ArmCommand (RoverFlake2/src/rover_msgs/msg/ArmCommand.msg).</summary>
    [Serializable, RosMsg("rover_msgs/msg/ArmCommand")]
    public class ArmCommand : RosMessage
    {
        public byte cmd_type;                          // ROS 'char' = uint8, e.g. 86 = 'V'
        public double[] positions = new double[0];
        public double[] velocities = new double[0];
        public double end_effector;
        public short cmd_value;
    }
}
