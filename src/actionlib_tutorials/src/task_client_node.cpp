#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/TaskAction.h>
#include <actionlib_tutorials/TaskActionFeedback.h>

void doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_tutorials::TaskResultConstPtr& result)
{
  ROS_INFO("処理完了");
  ROS_INFO("State [%s]", state.toString().c_str());
  ros::shutdown();
}

void activeCb()
{
  //Goalはactiveです
  ROS_INFO("Goal is active %d", ros::Time::now().sec);
}

// Called every time feedback is received for the goal
void feedbackCb(const actionlib_tutorials::TaskFeedbackConstPtr& feedback)
{
  //フィードバックデータ受信
  ROS_INFO("feedbackdata:：seqno %d", feedback->sequence);
  ROS_INFO("feedbackdata:：posx %f" , feedback->robot_arm_pos_x);
  ROS_INFO("feedbackdata:：posy %f" , feedback->robot_arm_pos_y);
  ROS_INFO("feedbackdata:：robot_state %d", feedback->robot_state);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "client");

  actionlib::SimpleActionClient<actionlib_tutorials::TaskAction> action_client("Task01", true);

  //ROS_INFO(" ** クライアントプログラムです **");
  //ROS_INFO("アクションサーバを待ちます...");
  ROS_INFO(" ** client **");
  ROS_INFO("waiting action server");

  // 接続待ち処理（組み込み）
  action_client.waitForServer();

  //ROS_INFO("アクションサーバが開始しました. goalを送ります");
  ROS_INFO("action server started. goal sending");
  actionlib_tutorials::TaskGoal goal;
  //目標値構造体の中に、値を設定
  //何回server処理実施するかを設定
  goal.order = 20;
  //目標値を設定
  goal.robot_arm_goal_x = 12.34;
  goal.robot_arm_goal_y = 43.21;

  // *** サーバ・アクションクライアントの同期実行の例 ***
  
  // action_client経由で、goalの処理要求を送出 
  // + 完了/生存確認/フィードバック用のコールバックを登録
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  // 周期処理
  ros::spin();

  //一回だけ実行する処理の例（コールバックを取らない、ブロック処理）
  //action_client.sendGoal(goal);
  //bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));

  //if (finished_before_timeout)
  //{
  //  actionlib::SimpleClientGoalState state = action_client.getState(); // ここでブロックされる
  //  ROS_INFO("アクション終了: %s",state.toString().c_str());
  //}
  //else
  //{
  //  ROS_INFO("タイムアウトしました");
  //  action_client.cancelGoal();
  //}

  return 0;
} 

