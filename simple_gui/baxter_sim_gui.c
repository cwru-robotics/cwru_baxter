//crude C code for GUI with OS service calls:
//compile with:
//gcc -o baxter_sim_gui baxter_sim_gui.c $(pkg-config --cflags --libs gtk+-2.0 gmodule-2.0)

//make sure the "glade" file, baxter_sim_gui.glade, and baxter_sim_gui.c are in the same dir;
//execute from this dir w/: ./baxter_sim_gui


#include <gtk/gtk.h>
#include <stdio.h>
#include <stdio.h> 
#include <stdlib.h>
/*

const int PCL_IDENTIFY_PLANE = 0;
const int PCL_FIND_PNTS_ABOVE_PLANE = 1;
const int PCL_COMPUTE_CYLINDRICAL_FIT_ERR_INIT = 2;
const int PCL_COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE = 3;
const int PCL_MAKE_CAN_CLOUD = 4;
const int PCL_FIND_ON_TABLE = 5;
const int PCL_TAKE_SNAPSHOT = 6;
*/

/* requests to coordinator:
const int RQST_DO_NOTHING = 0;
const int RQST_DISPLAY_REACHABILITY_AT_MARKER_HEIGHT = 1;
const int RQST_COMPUTE_MOVE_ARM_TO_PRE_POSE = 2;
const int RQST_MOVE_ARM_TO_APPROACH_POSE = 3;
const int RQST_MOVE_ARM_TO_GRASP_POSE = 4;
const int RQST_MOVE_ARM_DEPART = 5;
const int RQST_MOVE_ARM_TO_MARKER = 6;
const int RQST_MOVE_MARKER_TO_GRASP_POSE = 7;
const int RQST_DO_PLANNED_PATH=8;
const int RQST_DESCEND_20CM=9;
const int RQST_ASCEND_20CM=10;
*/


void 
on_window_destroy (GtkObject *object, gpointer user_data)
{
        gtk_main_quit();
}

G_MODULE_EXPORT void enable_control_cb(GtkButton *enable_control, gpointer data) 
{
  printf("enabling motor control\n");
    //system("cd ~/home/ros_ws");
    system("rosrun baxter_tools enable_robot.py -e");

}

G_MODULE_EXPORT void enable_gripper_usb_cb(GtkButton *enable_gripper_usb, gpointer data) 
{
  printf("enabling /dev/ttyUSB0\n");
    //system("cd ~/home/ros_ws");
    system("echo $PASSWORD | sudo -S chmod ugo+rwx /dev/ttyUSB0");

}

G_MODULE_EXPORT void take_snapshot_cb(GtkButton *take_snapshot, gpointer data) 
{
  printf("refreshing snapshot of pointcloud\n");
    system("rosservice call pcl_perception_svc  6");
}

G_MODULE_EXPORT void find_plane_cb(GtkButton *find_plane, gpointer data) 
{
  printf("finding plane based on selected patch\n");
    system("rosservice call pcl_perception_svc  0");
}

G_MODULE_EXPORT void find_points_above_plane_cb(GtkButton *find_points_above_plane, gpointer data) 
{
  printf("finding points above plane\n");
    system("rosservice call pcl_perception_svc 1");
}

G_MODULE_EXPORT void initial_model_fit_cb(GtkButton *initial_model_fit, gpointer data) 
{
  printf("initial model fit\n");
    system("rosservice call pcl_perception_svc 2");
}

G_MODULE_EXPORT void iterate_model_fit_cb(GtkButton *iterate_model_fit, gpointer data) 
{
  printf("iterate model fit\n");
    system("rosservice call pcl_perception_svc 3");
}

G_MODULE_EXPORT void move_IM_to_grasp_pose_cb(GtkButton *move_IM_to_grasp_pose, gpointer data) 
{
  printf("move IM to grasp pose: \n");
  system("rosservice call coordinator_svc 7");

}

G_MODULE_EXPORT void compute_reachability_cb(GtkButton *display_reachability, gpointer data) 
{
  printf("compute and display reachability at marker height\n");
  system("rosservice call coordinator_svc 1");
}

G_MODULE_EXPORT void plan_to_pre_pose_cb(GtkButton *move_to_pre_pose, gpointer data) 
{
  printf("computing motion plan to waiting pose\n");
  system("rosservice call coordinator_svc 2");
}

//compute_jspace_move_to_waiting_pose_cb
G_MODULE_EXPORT void compute_jspace_move_to_waiting_pose_cb(GtkButton *compute_jspace_move_to_waiting_pose, gpointer data) 
{
  printf("computing jspace motion plan to waiting pose\n");
  system("rosservice call coordinator_svc 11");
}

G_MODULE_EXPORT void plan_descend_20cm_cb(GtkButton *plan_descend_20cm, gpointer data) 
{
  printf("computing motion for 20cm descent\n");
  system("rosservice call coordinator_svc 9");
}

G_MODULE_EXPORT void plan_ascend_20cm_cb(GtkButton *plan_ascend_20cm, gpointer data) 
{
  printf("computing motion plan to ascend 20cm\n");
  system("rosservice call coordinator_svc 10");
}
//find_beercan_grasp_pose_cb
G_MODULE_EXPORT void find_beercan_grasp_pose_cb(GtkButton *find_beercan_grasp_pose, gpointer data) 
{
  printf("find beercan grasp pose\n");
  system("rosservice call coordinator_svc 14");
}

G_MODULE_EXPORT void plan_move_to_IM_cb(GtkButton *plan_move_to_IM, gpointer data) 
{
  printf("computing motion plan to Interactive Marker\n");
  system("rosservice call coordinator_svc 6");
}

G_MODULE_EXPORT void preview_move_cb(GtkButton *preview_move, gpointer data) 
{
  printf("previewing motion plan\n");
  system("rosservice call coordinator_svc 12");
}

G_MODULE_EXPORT void execute_move_cb(GtkButton *execute_move, gpointer data) 
{
  printf("executing motion plan\n");
  system("rosservice call coordinator_svc 8");
}

G_MODULE_EXPORT void open_gripper_cb(GtkButton *open_gripper, gpointer data) 
{
  printf("opening gripper\n");
  system("rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -4.0"); //angle pi rot of motor--pretty much fully open fingers
}

G_MODULE_EXPORT void close_gripper_cb(GtkButton *close_gripper, gpointer data) 
{
  printf("closing gripper\n");
  system("rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -1.1"); // angle 90deg rot of motor; should be OK to pick up can
}


int
main (int argc, char *argv[])
{
        GtkBuilder              *builder;
        GtkWidget               *window;
     	GError     *error = NULL;       
        gtk_init (&argc, &argv);
        
        builder = gtk_builder_new ();

/*
    if( ! gtk_builder_add_from_file( builder, "service_btns.glade", &error ) )
    {
        g_warning( "%s", error->message );
        g_free( error );
        return( 1 );
    }
*/
        gtk_builder_add_from_file (builder, "baxter_sim_gui.glade", NULL);
        /* window = GTK_WIDGET (gtk_builder_get_object (builder, "window")); */
        window = GTK_WIDGET (gtk_builder_get_object (builder, "window1"));
        gtk_builder_connect_signals (builder, NULL);          
        g_object_unref (G_OBJECT (builder));
        
        gtk_widget_show (window);       
        gtk_main ();
          return 0;
}      
    
