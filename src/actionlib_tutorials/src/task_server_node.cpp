#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/TaskAction.h>

class TaskAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::TaskAction> action_server_entity; 
  std::string action_name_;
  actionlib_tutorials::TaskFeedback feedback_;
  actionlib_tutorials::TaskResult result_;
  float robotarm_x;
  float robotarm_y;

public:

  TaskAction(std::string name) :
    action_server_entity(nh_, name, boost::bind(&TaskAction::executeCB, this, _1), false),
    action_name_(name)
  {
    float robotarm_x = 0.0;
    float robotarm_y = 0.0;
    action_server_entity.start();
  }

  ~TaskAction(void)
  {
  }

  uint8_t setRobotState(int input)
  {
    return (uint8_t)( input % 3);
  }

  void executeCB(const actionlib_tutorials::TaskGoalConstPtr &goal)
  {
    ros::Rate r(1);
    bool success = true;
    float start_goal_diff_x = 0.0;
    float start_goal_diff_y = 0.0;
    feedback_.sequence = goal->sequence;
    result_.complete = false;

    start_goal_diff_x = goal->robot_arm_goal_x - robotarm_x;
    start_goal_diff_y = goal->robot_arm_goal_y - robotarm_y;
 
    // goal内で設定されたorder数だけ実施(clientで規定)
    for(int i=1; i<=goal->order; i++)
    {
      // (制御処理を模擬)
      robotarm_x += start_goal_diff_x / (goal->order);
      robotarm_y += start_goal_diff_y / (goal->order);

      // action server / rosエラー検出
      if (action_server_entity.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        action_server_entity.setPreempted();
        success = false;
        break;
      }
      //フィードバックデータ内(Task.actionで規定)のsequenceが増加する
      feedback_.sequence++;
      //現在の腕の位置
      feedback_.robot_arm_pos_x = robotarm_x;
      feedback_.robot_arm_pos_y = robotarm_y;
      //他、必要なデータを付与する
      feedback_.robot_state = setRobotState((int) ros::Time::now().sec);
      action_server_entity.publishFeedback(feedback_);
      r.sleep();
    }

    if(success)
    {
      result_.complete = true;
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      action_server_entity.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Task01");

  TaskAction Task("Task01");
  ros::spin();

  return 0;
}
