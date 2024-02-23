/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosUtilities.MsgHandling;
using UnityStandardAssets.Vehicles.Car;

namespace RosUtilities.Communication
{
    public class TwistPublisher : MonoBehaviour
    {
        public CarController m_Car; // the car controller we want to use
        private RosMessageTypes.Geometry.TwistStampedMsg message;
        private float previousRealTime;        

        ROSConnection ros;
        public string topicName = "cmd_vel";        

        protected void Start()
        {
            Debug.Log("__ImagePublisher START");
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<TwistStampedMsg>(topicName);

            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new RosMessageTypes.Geometry.TwistStampedMsg();
        }
        private void UpdateMessage()
        {
            message.twist.linear.x = m_Car.CurrentSpeed / 2.23693629f; // MPH -> m/s.

            ros.Publish(topicName, message);
        }
    }
}
