// #define FRAMERATE 30;

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;



public class testnode : MonoBehaviour
{
   

    private ROS2Node hello_world_node;
    private IPublisher<std_msgs.msg.String> chatter_pub;
    private int FRAMERATE = 30;
    private int framecount = 0;
 // Start is called before the first frame update
    void Start()
    {
        ROS2UnityCore ros2Unity = new ROS2UnityCore();
        if (ros2Unity.Ok()) {
            hello_world_node = ros2Unity.CreateNode("ROS2UnityListenerNode");
                chatter_pub = hello_world_node.CreatePublisher<std_msgs.msg.String>("chatter"); 
        }
    }

    // Update is called once per frame
    void Update()
    {
        framecount++;
        if(framecount > FRAMERATE){
                   std_msgs.msg.String msg = new std_msgs.msg.String();
    msg.Data = "Hello Ros2ForUnity!";
    chatter_pub.Publish(msg);
    framecount = 0;
        }
 
        
    }
};
