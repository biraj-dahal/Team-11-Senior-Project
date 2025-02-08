# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).


Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

## Notes On This Fork ##

This fork has implemented changes to the repo to enable building to UWP devices, like the Microsoft HoloLens. Like the main ROS# branch, use 2019.x or later.

#### Installation ### 

To use ROS# with the HoloLens, simply clone this fork and stay on the master branch. Then open the Unity project and import the [Microsoft Mixed Reality Toolkit](https://github.com/Microsoft/MixedRealityToolkit-Unity). The toolkit provides a version of Newtonsoft.Json that works on the HoloLens, as well as other tools/features you will want during HoloLens development. Follow the Mixed Reality Toolkit configuration instructions, and you will be good to go. I use the 2017 version of MRTK, not vNext.

### Architecture ###

How does this work under the hood? In brief, I wrote a UWP-compatible WebSocket interface for ROS#, created a UWP-compatible version of RosBridgeClient.dll, called RosBridgeClientUWP.dll, added that to the Unity Project, specified proper platforms for all .dlls, and edited RosConnector.cs to automatically use to the UWP WebSocket interface.

#### Creating RosBridgeClientUWP.dll ####

ROS# contains a solution in the Libraries folder, which contains a project called RosBridgeClient. In the Protocols folder, I created a UWP compatible WebSocket interface. Next, I wrapped all WebSocket protocol files in preprocessor directives so only the UWP compatible interface would be compiled in a UWP-build. Finally, I created a second project in the solution called RosBridgeClientUWP. I made it a Windows Universal class library project, and copied all of the RosBridgeClient code over as links. Copying as links means that editing the code in one location changes it in both. Finally, I built the solution.

#### Preparing the Unity Project ####

In the ROS# Unity project, I first installed the Mixed Reality Toolkit and followed their configuration instructions. Next, in RosSharp/Plugins, I deselected all platforms from Newtonsoft.Json, and excluded WSAPlayer from all others. Next, I copied over the RosBridgeClientUWP.dll into RosSharp/Plugins, and selected WSAPlayer as the only platform. Finally, I modified RosConnector.cs, using preprocessor directives to make Unity use the WebSocketUWP protocol if WINDOWS_UWP is defined.

### Making Changes ###

If you want to make changes to the RosBridgeClient, like adding new messages, for instance, simply edit the code in the RosBridgeClient project (following the instructions from the main ROS# wiki), build the solution, and copy over the new RosBridgeClient.dll and RosBridgeClientUWP.dll.


### Compatibile With Mixed Reality Toolkit ###
This branch is compatible with Microsoft's Mixed Reality Toolkit. See the Preparing Unity Project Section.

## Recent Changes ##

[This](https://github.com/siemens/ros-sharp/commit/acdd1ea7b8de47a23fbf376fa590590cf945b495) commit comes with major changes in how ROS# deals with URDF import/export

The biggest changes are:
* [Urdf Libary](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf): The UrdfImporter project was renamed to Urdf. It now supports the ability to both read from and write to URDF files.
* [Create, Modify, and Export URDF models in Unity](https://github.com/siemens/ros-sharp/tree/master/Unity3D): ROS# now supports creating and exporting URDF models directly in Unity. It is also possible to modify and re-export an existing URDF model.
* [Transfer URDF files from Unity to ROS](https://github.com/siemens/ros-sharp/wiki/User_App_ROS_TransferURDFToROS): Previously it was only possible to transfer/import URDF files from ROS to Unity. Now ROS# can send a URDF and all its meshes from Unity to a package in ROS.

Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki/), especially [Section 3.2](https://github.com/siemens/ros-sharp/wiki/User_App_NoROS_ExportURDFOnWindows), for an explanation of how to use the new framework.



## Contents ##

* [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries): .NET solution for
[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient),
[Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf) and
[MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration)
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS):  [ROS](http://wiki.ros.org/) packages used by ROS#.
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): [Unity](https://unity3d.com/) project containing
  * Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and
   [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter) and
   [MessageGeneration](https://github.com/siemens/ros-sharp/tree/master/Libraries/MessageGeneration)
  * example scenes and reference code (see [Wiki](https://github.com/siemens/ros-sharp/wiki))

## Releases ##

In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported in other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
  * to be used in other .NET projects using these libraries.

The latest release is also being published in the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-ros-unity-communication-package-107085).

Please get the latest development version directly from the [tip of this master branch](https://github.com/siemens/ros-sharp).

## Unity Robotics Hub ##
In 11/2020 Unity launched [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) and included a major part of ROS#.
* [Short Description](https://github.com/siemens/ros-sharp/wiki/Ext_UnityRoboticsHub) on Unity Robotics Hub written by Unity developers
* [Differences](https://github.com/siemens/ros-sharp/wiki/Ext_RosSharp_RoboticsHub) between ROS# and Unity Robotics Hub

We are in close contact with the developers and decided to run both initiatives in parallel.

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:
* [Newtonsoft.Json](https://github.com/JamesNK/Newtonsoft.Json) (MIT License)
* [Newtonsoft.Json.Bson](https://github.com/JamesNK/Newtonsoft.Json.Bson) (MIT License)
* [websocket-sharp](https://github.com/sta/websocket-sharp) (MIT License), required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on a variety of other platforms community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) Visual Studio solution requires .NET Framework 4.6.1 and Visual Studio 2017 or higher.
* The Unity Project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) has been developed with Unity Version 2019.4.18f (LTS) and should be compatible also with older versions.
In Versions below 2019.3, make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* Please find a UWP version of ROS# [here](https://github.com/EricVoll/ros-sharp).
* Please find a .NET Standard 2.0 version of UrdfImporter [here](https://github.com/blommers/UdrfImporter).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2021

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
