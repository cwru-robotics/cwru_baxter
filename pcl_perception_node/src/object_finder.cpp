const double TABLE_VIEW_X_MIN = 0.35; //0.3 picks up some of e-stop box
const double TABLE_VIEW_X_MAX = 0.9;
const double TABLE_VIEW_Y_MIN = -0.2;
const double TABLE_VIEW_Y_MAX = 0.2;

//const double default_table_z_height_wrt_baxter_base_frame = -0.2; // replace this with data from perception...later


class Object_finder {
public:
    Object_finder(); //constructor
    // special-purpose fncs to find grasp frame of specific objects, given pointcloud
    // lots of restrictions/assumptions apply, specialized for various objects
    // the point-cloud provided is assumed to be w/rt the Baxter base frame

    // since multiple grasp frames may exist, a vector of transforms is filled;
    // these correspond to options for yale-hand grasp frames w/rt Baxter base frame
    
    //oops--perception node should not be responsible for assigning grasp frames--only for identifying object frames
    //void find_coke_can_grasps_from_above(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, double table_height, std::vector<Eigen::Affine3d> &grasp_xforms);
    Eigen::Affine3d find_coke_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, double table_height);
    // add lots more special-purpose fncs here...

private:
    //helper and specialized functions
    //find the model frame for a defined coke can; express as xform of model frame w/rt baxter base frame
    //Eigen::Affine3d find_model_frame_coke_can(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, double table_height)

    // given model frame, expressed as a xform w/rt Baxter base frame, 
    // fill in a list of viable grasp frames--also expressed as Affine xforms of Yale-hand frame w/rt Baxter base frame:
    // expressed as a PoseStamped (frame_id should be Baxter base frame)
    //grasps_coke_can_from_above(std::vector<Eigen::Affine3d> &grasp_xforms);

    // find pts within a specified "box"; return list of indices of pts inside the box
    void box_filter(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, vector<int> &indices);

    // a centroid function; will operate only on the points listed by index in iselect;
    // resulting vector is in same frame (baxter base) as point cloud
    Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect);
    
   // for set of points nearly in horizontal plane, project these points to an x-y plane, then find a best fit of a circle of given radius to these points;
    //  fill in the x,  y, z components of model_frame_origin   
    void fit_horizontal_circle_to_points(double radius, PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> indices, Eigen::Vector3d & model_frame_origin);

}; //end of class definition; later, move this to a header file

Object_finder::Object_finder() { // constructor
    // do initializations here
}


Eigen::Vector3f Object_finder::computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;
    int nselect = iselect.size();
    for (int i = 0; i < nselect; i++) {
        centroid += pcl_cloud->points[iselect[i]].getVector3fMap();
    }
    if (nselect > 0) {
        centroid /= ((float) nselect);
    }
    return centroid;
}

void Object_finder::box_filter(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double x_min, double x_max, double y_min, double y_max,
        double z_min, double z_max, vector<int> &indices) {

    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    //double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        //dz = pt[2] - z_threshold;
        if ((pt[2] > z_min)&&(pt[2] < z_max)) {
            if ((pt[0] > x_min)&&(pt[0] < x_max)&&(pt[1] > y_min)&&(pt[1] < y_max)) {
                indices.push_back(i);
                //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
                //cin>>ans;
            }
        }
    }
    int n_extracted = indices.size();
    cout << " number of points extracted = " << n_extracted << endl;
    
    //FOR DEBUG:  this option will likely go away when g_pckKinect is inaccessible
    copy_cloud(g_pclKinect, indices, g_display_cloud); //g_display_cloud is being published regularly by "main"
}

void Object_finder::fit_horizontal_circle_to_points(double template_radius, PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> indices, Eigen::Vector3d & model_frame_origin) {
    //seed the soln w/ computation of centroid
    Eigen::Vector3f approx_center;
    approx_center = computeCentroid(pt_cloud_wrt_base, indices);
    // could perturb center and count number of overlapping points, using pt_radius = (pt - approx_center).norm(), and test for pt_radius< template_radius
    // return soln that captures the most points??
    // also look for expectation of non-pts?
    // try using template-fit approach?  create discretized grid and seek 1/0 consistency?
    // TO-DO
    for (int i=0;i<3;i++) {
       model_frame_origin(i) = approx_center(i);
    }
}

Eigen::Affine3d Object_finder::find_coke_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, double table_height) {
    //coke can dimensions, per Kinect data
    const double RADIUS = 0.03; //estimated from ruler tool...example to fit a cylinder of this radius to data
    const double H_CYLINDER = 0.12; // estimated height of cylinder
    const double GRASP_V_OFFSET = 0.1; // TUNE THIS: appropriate vertical offset for can grasp from above
    double z_min = table_height + H_CYLINDER - 0.01;
    double z_max = table_height + H_CYLINDER + 0.01;
    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Eigen::Affine3d affine_model_frame_wrt_base;
    

    vector<int> indices;
    Eigen::Vector3d model_frame_origin;

    // fit model to points; identify model frame:
    //find points within a thin, horizontal slab bounding expected height of can
    box_filter(pt_cloud_wrt_base, TABLE_VIEW_X_MIN, TABLE_VIEW_X_MAX, TABLE_VIEW_Y_MIN, TABLE_VIEW_Y_MAX, z_min, z_max, indices);
    // project these points to an x-y plane, then find a best fit of a circle of given radius to these points;
    //  fill in the x and y components
    fit_horizontal_circle_to_points(RADIUS, pt_cloud_wrt_base, indices, model_frame_origin);
    model_frame_origin(2) = table_height; // coke can frame origin defined at base of can, i.e. at table height    
    
    // represent this as an affine matrix
    affine_model_frame_wrt_base.linear() = R;
    affine_model_frame_wrt_base.translation() = model_frame_origin;

    return affine_model_frame_wrt_base;
    
}

/*
    Eigen::Vector3d grasp_origin_wrt_model_origin;
    grasp_origin_wrt_model_origin(0) = 0.0;
    grasp_origin_wrt_model_origin(1) = 0.0;
    grasp_origin_wrt_model_origin(2) = H_CYLINDER + GRASP_V_OFFSET;

    grasp_frame_wrt_model_frame.linear() = R; // align gripper frame w/ model frame...many more options exist
    grasp_frame_wrt_model_frame.translation() = grasp_origin_wrt_model_origin; // vertical offset for grasp from above

    // apply transform to get corresponding grasp frame:
    // sup{base}_A_sub{grasp} = sup{base}_A_sub{model}*sup{model}_A_sub{grasp}
    affine_grasp_wrt_base = affine_model_frame_wrt_base*grasp_frame_wrt_model_frame;

    // could put in lots more options, but we'll just return this one option for now:
    grasp_xforms.clear();
    grasp_xforms.push_back(affine_grasp_wrt_base);
*/

