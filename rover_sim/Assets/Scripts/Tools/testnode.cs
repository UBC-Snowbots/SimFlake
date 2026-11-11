using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Connection test. Publishes "Hello from SimFlake N" on /chatter once a second and logs anything
/// received on /listener. Try:  ros2 topic echo /chatter   and
///   ros2 topic pub /listener std_msgs/msg/String "{data: hi}" --once
/// </summary>
public class testnode : MonoBehaviour
{
    public string chatterTopic = "/chatter";
    public string listenerTopic = "/listener";
    public float publishPeriod = 1f;

    RosPublisher<std_msgs.msg.String> chatter;
    float timer;
    int count;

    void Start()
    {
        chatter = Ros.Advertise<std_msgs.msg.String>(chatterTopic);
        Ros.Subscribe<std_msgs.msg.String>(listenerTopic, msg => Debug.Log($"[testnode] heard: {msg.data}"));
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < publishPeriod) return;
        timer = 0;
        chatter.Publish(new std_msgs.msg.String { data = $"Hello from SimFlake {count++}" });
    }
}
