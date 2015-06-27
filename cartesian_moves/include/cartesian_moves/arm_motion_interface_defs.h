#ifndef ARM_MOTION_INTERFACE_DEGS_H_
#define ARM_MOTION_INTERFACE_DEGS_H_
const int ARM_TEST_MODE =0;
//queries: 
const int ARM_IS_SERVER_BUSY_QUERY = 1;
const int ARM_QUERY_IS_PATH_VALID = 2;
const int ARM_GET_Q_DATA = 3;


//requests for motion plans:
const int ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE=20; //plan paths from current arm pose
const int ARM_PLAN_PATH_CURRENT_TO_PRE_POSE=21;


const int ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE=22;
const int ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL=23;

const int ARM_PLAN_PATH_QSTART_TO_QGOAL = 25;
const int ARM_PLAN_PATH_QSTART_TO_ADES = 24; //specify start and end, j-space or hand pose

const int ARM_PLAN_PATH_ASTART_TO_QGOAL = 26;


//MOVE command!
const int ARM_EXECUTE_PLANNED_PATH = 100;
const int ARM_DESCEND_20CM=101;
const int ARM_DEPART_20CM=102;

//response codes...
const int ARM_STATUS_UNDEFINED=0;
const int ARM_RECEIVED_AND_INITIATED_RQST=1;
const int ARM_REQUEST_REJECTED_ALREADY_BUSY=2;
const int ARM_SERVER_NOT_BUSY=3;
const int ARM_SERVER_IS_BUSY=4;
const int ARM_RECEIVED_AND_COMPLETED_RQST=5;
const int ARM_PATH_IS_VALID=6;
const int ARM_PATH_NOT_VALID=7;
#endif
