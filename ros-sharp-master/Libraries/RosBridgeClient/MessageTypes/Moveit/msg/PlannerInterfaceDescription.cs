/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



namespace RosSharp.RosBridgeClient.MessageTypes.Moveit
{
    public class PlannerInterfaceDescription : Message
    {
        public const string RosMessageName = "moveit_msgs/PlannerInterfaceDescription";

        //  The name of the planner interface
        public string name { get; set; }
        //  The names of the planner ids within the interface
        public string[] planner_ids { get; set; }

        public PlannerInterfaceDescription()
        {
            this.name = "";
            this.planner_ids = new string[0];
        }

        public PlannerInterfaceDescription(string name, string[] planner_ids)
        {
            this.name = name;
            this.planner_ids = planner_ids;
        }
    }
}
