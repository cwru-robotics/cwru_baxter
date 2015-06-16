// cart_traj_action_client: 
// wsn, June, 2015
// illustrates use of cart_move_hand_down_as, action server called "cartTrajActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<cartesian_moves/cartTrajAction.h>

using namespace std;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const cartesian_moves::cartTrajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val traj_id = %d", result->traj_id);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cart_traj_action_client_node"); // name this node 
    //int g_count = 0;
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    cartesian_moves::cartTrajGoal goal;

    // use the name of our server, which is: example_action (named in example_action_server.cpp)
    actionlib::SimpleActionClient<cartesian_moves::cartTrajAction> action_client("cartTrajActionServer", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        //return 0; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever


    ROS_INFO("connected to action server"); // if here, then we connected to the server;
    int traj_id = 0;
    double x, y, z;
    while (ros::ok()) {

        // stuff a goal message:
        cout<<"enter x,y,z of goal: e.g., (0.4, -0.6, 0.4) or (0.5, 0, 0.4) "<<endl;
        cout << "enter goal x value: ";
        cin>>x;
        cout << "enter goal y value: ";
        cin>>y;
        cout << "enter goal z value: ";
        cin>>z;
        goal.x = x;
        goal.y = y;
        goal.z = z;
        goal.speed = 1.0; // arbitrary for now--not yet used

        traj_id++;
        goal.traj_id = traj_id; // this merely sequentially numbers the goals sent
        ROS_INFO("sending traj_id %d", traj_id);
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(20.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for traj number %d", traj_id);
            return 0;
        } else {
            ROS_INFO("finished before timeout");
        }

    }

    return 0;
}

