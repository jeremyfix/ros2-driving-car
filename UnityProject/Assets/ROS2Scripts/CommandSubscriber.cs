using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosCommand = RosMessageTypes.DrivingUnityInterface.CarCommandMsg;

public class CommandSubscriber : MonoBehaviour
{
    public UnityStandardAssets.Vehicles.Car.CarRemoteControl remote_control;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosCommand>("command", CommandReceived);
    }

    void CommandReceived(RosCommand msg)
    {
        remote_control.Acceleration = (float)msg.throttle;
        remote_control.SteeringAngle = (float)msg.steering;
    }
}