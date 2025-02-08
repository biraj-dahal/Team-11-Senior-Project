/*
© Siemens AG, 2017-2019
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

using System;
using System.Threading;
using RosSharp.RosBridgeClient.Protocols;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        public int SecondsTimeout = 10;

        public RosSocket RosSocket { get; private set; }
<<<<<<< HEAD
        public enum Protocols { WebSocketSharp, WebSocketNET, WebSocketUWP };
        public RosBridgeClient.RosSocket.SerializerEnum Serializer;
        public Protocols Protocol;
=======
        public RosSocket.SerializerEnum Serializer;
        public Protocol protocol;
>>>>>>> 6e91309b2447c0919d3dabaa0650bb71e3e490dd
        public string RosBridgeServerUrl = "ws://192.168.0.1:9090";

        public ManualResetEvent IsConnected { get; private set; }

        public virtual void Awake()
        {
<<<<<<< HEAD
#if WINDOWS_UWP
            ConnectAndWait();
#else
=======
            IsConnected = new ManualResetEvent(false);
>>>>>>> 6e91309b2447c0919d3dabaa0650bb71e3e490dd
            new Thread(ConnectAndWait).Start();
#endif
        }

        protected void ConnectAndWait()
        {
            RosSocket = ConnectToRos(protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);

            if (!IsConnected.WaitOne(SecondsTimeout * 1000))
                Debug.LogWarning("Failed to connect to RosBridge at: " + RosBridgeServerUrl);
        }

        public static RosSocket ConnectToRos(Protocol protocolType, string serverUrl, EventHandler onConnected = null, EventHandler onClosed = null, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.Microsoft)
        {
            IProtocol protocol = ProtocolInitializer.GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;

<<<<<<< HEAD
            return new RosSocket(protocol,serializer);
        }

        private static RosBridgeClient.Protocols.IProtocol GetProtocol(Protocols protocol, string rosBridgeServerUrl)
        {

#if WINDOWS_UWP
                return new RosBridgeClient.Protocols.WebSocketUWPProtocol(rosBridgeServerUrl);
#else
            switch (protocol)
            {
                case Protocols.WebSocketNET:
                    return new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);
                case Protocols.WebSocketSharp:
                    return new RosBridgeClient.Protocols.WebSocketSharpProtocol(rosBridgeServerUrl);
                case Protocols.WebSocketUWP:
                    Debug.Log("WebSocketUWP only works when deployed to HoloLens, defaulting to WebSocketNetProtocol");
                    return new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);
                default:
                    return null;
            }
#endif
=======
            return new RosSocket(protocol, serializer);
>>>>>>> 6e91309b2447c0919d3dabaa0650bb71e3e490dd
        }

        private void OnApplicationQuit()
        {
            RosSocket.Close();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            IsConnected.Set();
            Debug.Log("Connected to RosBridge: " + RosBridgeServerUrl);
        }

        private void OnClosed(object sender, EventArgs e)
        {
            IsConnected.Reset();
            Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl);
        }
    }
}
