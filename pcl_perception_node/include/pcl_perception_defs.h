#ifndef PCL_PERCEPTION_DEFS_H_
#define PCL_PERCEPTION_DEFS_H_

const int IDENTIFY_PLANE = 0;
const int FIND_PNTS_ABOVE_PLANE = 1;
const int COMPUTE_CYLINDRICAL_FIT_ERR_INIT = 2;
const int COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE = 3;
//const int MAKE_CAN_CLOUD = 4;
const int FIND_ON_TABLE = 5;

const double Z_EPS = 0.01; //choose a tolerance for plane fitting, e.g. 1cm
const double R_EPS = 0.05; // choose a tolerance for cylinder-fit outliers

const double R_CYLINDER = 0.085; //estimated from ruler tool...example to fit a cyclinder of this radius to data
const double H_CYLINDER = 0.3; // estimated height of cylinder
#endif
