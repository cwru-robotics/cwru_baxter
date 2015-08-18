const double TABLE_VIEW_X_MIN = 0.35; //0.35; //0.3 picks up some of e-stop box
const double TABLE_VIEW_X_MAX = 1;
const double TABLE_VIEW_Y_MIN = -0.4;
const double TABLE_VIEW_Y_MAX = 0.4;

//search ranges for floor and table, from base-frame coords
// for gazebo model: fine search: 109382 floor points between z = -0.940000 and -0.929000 
const double FLOOR_SEARCH_MIN_Z = -1.5;
const double FLOOR_SEARCH_MAX_Z = -0.5;

const double FULL_SEARCH_MIN_Z = -1.5;
const double FULL_SEARCH_MAX_Z = 1.5;
const double DZ_SLAB_5CM = 0.05;
const double DZ_SLAB_1CM = 0.01;

//GAZEBO cafe table fine search: 143467 surface points between z = -0.160000 and -0.149000
const double TABLE_SEARCH_MIN_Z = -0.5;
const double TABLE_SEARCH_MAX_Z = 0.0;


const double R_GAZEBO_BEER = 0.055; //estimated from ruler tool...example to fit a cylinder of this radius to data
const double H_GAZEBO_BEER = 0.23; // estimated height of cylinder

const double H_COKE_CAN = 0.12;
const double R_COKE_CAN = 0.03;

//const double default_table_z_height_wrt_baxter_base_frame = -0.2; // replace this with data from perception...later
const double default_cafe_table_z_height_wrt_baxter_base_frame = -0.154706; //-0.213; // experimentally, this is height of stool relative to baxter frame;
// OK for default, but more generally, replace this with data from perception in calls to find objects
// gazebo: table height w/rt base = -0.154706

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
    Eigen::Affine3d find_coke_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> &indices_surface_optimal,double table_height); 
    
    // add lots more special-purpose fncs here...
    Eigen::Affine3d find_gazebo_beer_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> &indices_surface_optimal,
        double table_height = default_cafe_table_z_height_wrt_baxter_base_frame);
    
    //well...models are so specific, grasp frames of models should be associated;
    Eigen::Affine3f get_grasp_transform_gazebo_beer_grasp_from_above();
   Eigen::Affine3f  get_grasp_transform_coke_can_grasp_from_above();
     
 
    // some special-case objects: floor, table, and other horizontal surfaces
    //start w/ this: sorts all points into thick horizontal slabs;
    // finds most likely slab for floor, within floor search range;
    // subdivides this thick slab; returns indices of these points and sets internal var z_floor_
    bool find_floor(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> &indices_floor,
            double dz_slab = DZ_SLAB_5CM, double z_search_min = FULL_SEARCH_MIN_Z, double z_search_max = FULL_SEARCH_MAX_Z);

    //find_surface() is general for horizontal surfaces
    int find_surface(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> &indices_table,
            double dz_slab, double z_search_min, double z_search_max, double &z_surface);
    // alt version--accepts list of indices
    int find_surface(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> input_indices, vector<int> &indices_surface,
        double dz_slab, double z_search_min, double z_search_max, double &z_surface);

    //customized to find table height; default search range, and sets internal var z_table_
    int find_table(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> &indices_table, double &z_table,
            double dz_slab = DZ_SLAB_1CM, double z_search_min = TABLE_SEARCH_MIN_Z, double z_search_max = TABLE_SEARCH_MAX_Z);

    double get_table_height() {
        return z_table_;
    }

    double get_floor_height() {
        return z_floor_;
    }

    double get_object_height() {
        return z_top_of_object_;
    }

   //helper functions...
    ///given a pointer to a point cloud and a list if indices of interest, find list of indices for which the points
    // lie within a prescribed x and y window
    void xy_window(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> input_indices, vector<int> &output_indices,
            double x_min = TABLE_VIEW_X_MIN, double x_max = TABLE_VIEW_X_MAX, double y_min = TABLE_VIEW_Y_MIN, double y_max = TABLE_VIEW_Y_MAX);
    void sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, double z_search_min, double z_search_max, double dz_slab,
            std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs);
    void sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> indices, double z_search_min, double z_search_max, double dz_slab,
            std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs);

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
    // alt version: use provided indices into inputCloud
    //void box_filter(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> input_indices, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, vector<int> &indices);

    // a centroid function; will operate only on the points listed by index in iselect;
    // resulting vector is in same frame (baxter base) as point cloud
    Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect);

    // for set of points nearly in horizontal plane, project these points to an x-y plane, then find a best fit of a circle of given radius to these points;
    //  fill in the x,  y, z components of model_frame_origin   
    void fit_horizontal_circle_to_points(double radius, PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> indices, Eigen::Vector3d & model_frame_origin);

    std::vector<vector<int> > indices_by_slab_;
    int i_best_slab_;
    //int n_thick_slabs_;
    vector<float> z_mins_;
    vector<float> z_maxs_;
    Eigen::Vector4f plane_params_;
    double z_floor_;
    double z_table_;
    double z_top_of_object_;

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
    ROS_INFO("box filter of %d pts", npts);
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

void Object_finder::xy_window(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> input_indices, vector<int> &output_indices,
        double x_min, double x_max, double y_min, double y_max) {

    int npts = input_indices.size();
    Eigen::Vector3f pt;
    output_indices.clear();
    //double dz;
    int ans;
    int index;
    ROS_INFO("window filter of %d pts", npts);
    for (int i = 0; i < npts; ++i) {
        index = input_indices[i];
        pt = inputCloud->points[index].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        //dz = pt[2] - z_threshold;
        if ((pt[0] > x_min)&&(pt[0] < x_max)&&(pt[1] > y_min)&&(pt[1] < y_max)) {
            output_indices.push_back(index);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = output_indices.size();
    cout << " number of points extracted = " << n_extracted << endl;

    //FOR DEBUG:  this option will likely go away when g_pckKinect is inaccessible
    //copy_cloud(g_pclKinect, output_indices, g_display_cloud); //g_display_cloud is being published regularly by "main"
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
    for (int i = 0; i < 3; i++) {
        model_frame_origin(i) = approx_center(i);
    }
}

/*
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
 * */

//need to generalize:  coke can and gazebo can have identical finder code, except for r and h dimensions

Eigen::Affine3d Object_finder::find_coke_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> &indices_surface_optimal,double table_height) {
    //coke can dimensions, per Kinect data
    const double RADIUS = R_COKE_CAN; //estimated from ruler tool...example to fit a cylinder of this radius to data
    const double H_CYLINDER = H_COKE_CAN; // estimated height of cylinder
    const double GRASP_V_OFFSET = 0.1; // TUNE THIS: appropriate vertical offset for can grasp from above
    const double Z_TOL = 0.05;
    // look for points +/- 1cm from expected can height
    // gazebo beer-can top is at z= 0.075
    double z_min_can_top = table_height + H_CYLINDER / 2.0; //- Z_TOL;
    double z_max_can_top = table_height + H_CYLINDER * 1.5; // + Z_TOL;
    ROS_INFO("looking for coke can");
    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Eigen::Affine3d affine_model_frame_wrt_base;


    vector<int> indices_of_slab;
    //vector<vector<int> > indices_thin_slabs;
    Eigen::Vector3d model_frame_origin;

    // fit model to points; identify model frame:
    //find points within a thin, horizontal slab bounding expected height of can

    //xy_window(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> input_indices, vector<int> &output_indices,double x_min, double x_max, double y_min, double y_max)   
    // use the points by slabs...
    // find first candidate slab:  
    int nslabs = indices_by_slab_.size();
    int islab_start = 0;
    for (int islab = 0; islab < nslabs; islab++) {
        if (z_maxs_[islab] > z_min_can_top) {
            //found a slab that bounds min z search height:
            islab_start = islab;
            break;
        }
    }
    int islab_last = islab_start;
    ROS_INFO("searching for can top: islab_start= %d", islab_start);
    for (int islab = islab_start; islab < nslabs; islab++) {
        if (z_mins_[islab] > z_max_can_top) {
            //found a slab that bounds min z search height:
            islab_last = islab;
            break;
        }
    }
    ROS_INFO("searching for can top: islab_last= %d", islab_last);
    
    //now, use each of these slabs (perhaps only 1) to find can top
    double z_min_slab = z_mins_[islab_start];
    double z_max_slab = z_maxs_[islab_last];
    vector <int> indices_slab; 
    vector <int> indices_surface;
    //vector <int> indices_surface_optimal;
    vector <int> indices_windowed_slab;
    int npts_can_top_optimal=0;
    int npts_can_top=0;
    double z_can_top=0.0;
    int npts_windowed;
    for (int islab=islab_start;islab<=islab_last;islab++) {
            indices_slab = indices_by_slab_[islab];
               //window this slab by x and y limits
            xy_window(pt_cloud_wrt_base, indices_slab, indices_windowed_slab);
            npts_windowed = indices_windowed_slab.size();
               //ROS_INFO("thick slab %d; indices_windowed_slab has %d points ", islab,npts_windowed);    
         
            //search for can top within this slab:
            npts_can_top = find_surface(pt_cloud_wrt_base, indices_windowed_slab, indices_surface, DZ_SLAB_1CM, z_min_can_top, z_max_can_top, z_top_of_object_);
            //ROS_INFO("thick slab %d;  best subslab has %d points at height %f", islab,npts_can_top, z_top_of_object_);    
            if (npts_can_top>npts_can_top_optimal) {
                indices_surface_optimal=indices_surface;
                npts_can_top_optimal = npts_can_top;
                z_can_top = z_top_of_object_;
            }   
    }
    z_top_of_object_ = z_can_top; //set internal var to best-guess can top height
    ROS_INFO("optimal subslab has %d points at height %f", npts_can_top_optimal, z_can_top);    
   
    

    // project these points to an x-y plane, then find a best fit of a circle of given radius to these points;
    //  fill in the x and y components
    fit_horizontal_circle_to_points(RADIUS, pt_cloud_wrt_base, indices_surface_optimal, model_frame_origin);
    model_frame_origin(2) = table_height; // coke can frame origin defined at base of can, i.e. at table height    

    // represent this as an affine matrix
    affine_model_frame_wrt_base.linear() = R;
    affine_model_frame_wrt_base.translation() = model_frame_origin;

    return affine_model_frame_wrt_base;

}


Eigen::Affine3d Object_finder::find_gazebo_beer_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, vector<int> &indices_surface_optimal,double table_height) {
    //coke can dimensions, per Kinect data
    const double RADIUS = R_GAZEBO_BEER; //estimated from ruler tool...example to fit a cylinder of this radius to data
    const double H_CYLINDER = H_GAZEBO_BEER; // estimated height of cylinder
    const double GRASP_V_OFFSET = 0.1; // TUNE THIS: appropriate vertical offset for can grasp from above
    const double Z_TOL = 0.05;
    // look for points +/- 1cm from expected can height
    // gazebo beer-can top is at z= 0.075
    double z_min_can_top = table_height + H_CYLINDER / 2.0; //- Z_TOL;
    double z_max_can_top = table_height + H_CYLINDER * 1.5; // + Z_TOL;
    ROS_INFO("looking for gazebo beer can");
    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Eigen::Affine3d affine_model_frame_wrt_base;


    vector<int> indices_of_slab;
    //vector<vector<int> > indices_thin_slabs;
    Eigen::Vector3d model_frame_origin;

    // fit model to points; identify model frame:
    //find points within a thin, horizontal slab bounding expected height of can

    //xy_window(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> input_indices, vector<int> &output_indices,double x_min, double x_max, double y_min, double y_max)   
    // use the points by slabs...
    // find first candidate slab:  
    int nslabs = indices_by_slab_.size();
    int islab_start = 0;
    for (int islab = 0; islab < nslabs; islab++) {
        if (z_maxs_[islab] > z_min_can_top) {
            //found a slab that bounds min z search height:
            islab_start = islab;
            break;
        }
    }
    int islab_last = islab_start;
    ROS_INFO("searching for can top: islab_start= %d", islab_start);
    for (int islab = islab_start; islab < nslabs; islab++) {
        if (z_mins_[islab] > z_max_can_top) {
            //found a slab that bounds min z search height:
            islab_last = islab;
            break;
        }
    }
    ROS_INFO("searching for can top: islab_last= %d", islab_last);
    
    //now, use each of these slabs (perhaps only 1) to find can top
    double z_min_slab = z_mins_[islab_start];
    double z_max_slab = z_maxs_[islab_last];
    vector <int> indices_slab; 
    vector <int> indices_surface;
    //vector <int> indices_surface_optimal;
    vector <int> indices_windowed_slab;
    int npts_can_top_optimal=0;
    int npts_can_top=0;
    double z_can_top=0.0;
    int npts_windowed;
    for (int islab=islab_start;islab<=islab_last;islab++) {
            indices_slab = indices_by_slab_[islab];
               //window this slab by x and y limits
            xy_window(pt_cloud_wrt_base, indices_slab, indices_windowed_slab);
            npts_windowed = indices_windowed_slab.size();
               //ROS_INFO("thick slab %d; indices_windowed_slab has %d points ", islab,npts_windowed);    
         
            //search for can top within this slab:
            npts_can_top = find_surface(pt_cloud_wrt_base, indices_windowed_slab, indices_surface, DZ_SLAB_1CM, z_min_can_top, z_max_can_top, z_top_of_object_);
            //ROS_INFO("thick slab %d;  best subslab has %d points at height %f", islab,npts_can_top, z_top_of_object_);    
            if (npts_can_top>npts_can_top_optimal) {
                indices_surface_optimal=indices_surface;
                npts_can_top_optimal = npts_can_top;
                z_can_top = z_top_of_object_;
            }   
    }
    z_top_of_object_ = z_can_top; //set internal var to best-guess can top height
    ROS_INFO("optimal subslab has %d points at height %f", npts_can_top_optimal, z_can_top);    
   
    

    // project these points to an x-y plane, then find a best fit of a circle of given radius to these points;
    //  fill in the x and y components
    fit_horizontal_circle_to_points(RADIUS, pt_cloud_wrt_base, indices_surface_optimal, model_frame_origin);
    model_frame_origin(2) = table_height; // coke can frame origin defined at base of can, i.e. at table height    

    // represent this as an affine matrix
    affine_model_frame_wrt_base.linear() = R;
    affine_model_frame_wrt_base.translation() = model_frame_origin;

    return affine_model_frame_wrt_base;

}

//specialized fnc: return an affine transform corresponding to grasp from from above w/rt gazebo beer can frame
// this is specific to a model, to a grasp strategy, and to a specific (Yale-hand) gripper
 Eigen::Affine3f Object_finder::get_grasp_transform_gazebo_beer_grasp_from_above() {
    float H_GAZEBO_BEER_GRASP_FROM_ABOVE_Z = H_GAZEBO_BEER+ 0.10; // TUNE THIS EXPERIMENTALLY
    Eigen::Matrix3f R;
    //not sure if this is correct; DO need z-axis pointing down; what direction of x-axis is desired?  towards torso?
    R << -1, 0, 0,
            0, 1, 0,
            0, 0, -1;


    Eigen::Vector3f grasp_frame_origin;
    grasp_frame_origin<<0,0,H_GAZEBO_BEER_GRASP_FROM_ABOVE_Z;
    
     Eigen::Affine3f affine_grasp_frame_wrt_model;
     affine_grasp_frame_wrt_model.linear() = R;
     affine_grasp_frame_wrt_model.translation() = grasp_frame_origin;
     return affine_grasp_frame_wrt_model;
}
 
 Eigen::Affine3f Object_finder:: get_grasp_transform_coke_can_grasp_from_above() {
     float H_COKE_CAN_GRASP_FROM_ABOVE_Z = H_COKE_CAN+ 0.08; // TUNE THIS EXPERIMENTALLY
    Eigen::Matrix3f R;
    //not sure if this is correct; DO need z-axis pointing down; what direction of x-axis is desired?  towards torso?
    R << -1, 0, 0,
            0, 1, 0,
            0, 0, -1;


    Eigen::Vector3f grasp_frame_origin;
    grasp_frame_origin<<0,0,H_COKE_CAN_GRASP_FROM_ABOVE_Z;
    
     Eigen::Affine3f affine_grasp_frame_wrt_model;
     affine_grasp_frame_wrt_model.linear() = R;
     affine_grasp_frame_wrt_model.translation() = grasp_frame_origin;
     return affine_grasp_frame_wrt_model;    
     
 }

// this version takes a whole cloud--no input indices

void Object_finder::sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, double z_search_min, double z_search_max, double dz_slab,
        std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs) {

    int npts = inputCloudWrtBase->width * inputCloudWrtBase->height;
    Eigen::Vector3f pt;
    int ans;
    ROS_INFO("input cloud has %d points", npts);
    //cout << "enter 1: ";
    //cin>>ans;
    // sort points from z_min to z_max in slabs of thickness dz_slab
    // allow overlap of z_overlap to avoid boundary issues
    double z_overlap = 0.02; // choose this much overlap
    int nslabs = 1 + (z_search_max - z_search_min + z_overlap) / dz_slab;
    ROS_INFO("sorting points into %d slabs", nslabs);
    //vector<float> z_mins;
    //vector<float> z_maxs;
    z_mins.resize(nslabs);
    z_maxs.resize(nslabs);
    z_mins[0] = z_search_min;
    z_maxs[0] = z_search_min + dz_slab + z_overlap;
    for (int i = 1; i < nslabs; i++) {
        z_mins[i] = z_mins[i - 1] + dz_slab;
        z_maxs[i] = z_mins[i] + dz_slab + z_overlap;
    }
    //classify each pt by slab membership
    //vector<vector<int> > indices_by_slab; 
    indices_by_slab.clear();
    indices_by_slab.resize(nslabs);
    //ROS_INFO("clearing out vector of vectors...");
    for (int i = 0; i < nslabs; i++) indices_by_slab[i].clear();
    double z_height;
    int i_slab;
    // given z-height, nominal slab number = floor((z_height-z_search_min)/dz_slab)
    // but also check above and below overlaps
    for (int ipt = 0; ipt < npts; ipt++) {
        pt = inputCloudWrtBase->points[ipt].getVector3fMap();
        //ROS_INFO(" pt %d: %f, %f, %f", ipt, pt(0), pt(1), pt(2));
        z_height = pt(2);
        if ((z_height > z_search_min)&&(z_height < z_search_max)) {
            if (z_height == z_height) { // watch out for nan?
                i_slab = floor((z_height - z_search_min) / dz_slab); //nom slab membership--but check above and below as well
                //cout << "islab = " << i_slab << endl;
                indices_by_slab[i_slab].push_back(ipt);
                if (i_slab < nslabs - 2) { // not the top slab--so look above
                    if (z_height > z_mins[i_slab + 1]) {
                        //pt also fits in overlap w/ next higher slab:
                        indices_by_slab[i_slab + 1].push_back(ipt);
                    }
                }
                //check below:
                if (i_slab > 0) { // not the bottom slab
                    if (z_height < z_maxs[i_slab - 1]) {
                        //pt also fits in overlap w/ next lower slab:
                        indices_by_slab[i_slab - 1].push_back(ipt);
                    }
                }
            } else {
                //ROS_INFO("pt %d: Nan!", ipt); //got LOTS of NaN's; (pts > 260,000); don't know why
            }
        } // done sorting points by slabs
    }
    // find the slab with the most points:
    i_best_slab = 0;
    int npts_islab = indices_by_slab[i_best_slab].size();
    int npts_best_slab = npts_islab;


    //ROS_INFO("npts slab %d = %d", i_best_slab, npts_best_slab);
    for (int islab = 1; islab < nslabs; islab++) {
        npts_islab = indices_by_slab[islab].size();
        //ROS_INFO("npts slab %d = %d", islab, npts_islab);
        if (npts_islab > npts_best_slab) {
            npts_best_slab = npts_islab;
            i_best_slab = islab;
        }
    }
    ROS_INFO("best slab num = %d with %d pts", i_best_slab, npts_best_slab);
}

// variant--operate only on listed points from pointcloud

void Object_finder::sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> indices, double z_search_min, double z_search_max, double dz_slab,
        std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs) {
    int npts = indices.size(); //inputCloudWrtBase->width * inputCloudWrtBase->height;
    Eigen::Vector3f pt;
    const double THIN_SLAB_OVERLAP=0.001; // tune this
    // sort points from z_min to z_max in slabs of thickness dz_slab
    // allow overlap of z_overlap to avoid boundary issues
    double z_overlap = THIN_SLAB_OVERLAP; // choose this much overlap
    int nslabs = 1 + (z_search_max - z_search_min + z_overlap) / dz_slab;
    //ROS_INFO("sorting points into %d slabs", nslabs);
    //vector<float> z_mins;
    //vector<float> z_maxs;
    z_mins.resize(nslabs);
    z_maxs.resize(nslabs);
    z_mins[0] = z_search_min;
    z_maxs[0] = z_search_min + dz_slab + z_overlap;
    for (int i = 1; i < nslabs; i++) {
        z_mins[i] = z_mins[i - 1] + dz_slab;
        z_maxs[i] = z_mins[i] + dz_slab + z_overlap;
    }
    //classify each pt by slab membership
    //vector<vector<int> > indices_by_slab; 
    indices_by_slab.clear();
    indices_by_slab.resize(nslabs);
    //ROS_INFO("clearing out vector of vectors...");
    for (int i = 0; i < nslabs; i++) indices_by_slab[i].clear();
    double z_height;
    int i_slab;
    // given z-height, nominal slab number = floor((z_height-z_search_min)/dz_slab)
    // but also check above and below overlaps
    int sel_pt;
    for (int ipt = 0; ipt < npts; ipt++) {
        sel_pt = indices[ipt];
        pt = inputCloudWrtBase->points[sel_pt].getVector3fMap();
        //ROS_INFO(" pt %d: %f, %f, %f",ipt,pt(0),pt(1),pt(2));
        z_height = pt(2);
        if (z_height == z_height) { // watch out for nan
            i_slab = floor((z_height - z_search_min) / dz_slab); //nom slab membership--but check above and below as well
            if ((i_slab >= 0)&&(i_slab < nslabs)) {
                indices_by_slab[i_slab].push_back(sel_pt);
                if (i_slab < nslabs - 2) { // not the top slab--so look above
                    if (z_height > z_mins[i_slab + 1]) {
                        //pt also fits in overlap w/ next higher slab:
                        indices_by_slab[i_slab + 1].push_back(sel_pt);
                    }
                }
                //check below:
                if (i_slab > 0) { // not the bottom slab
                    if (z_height < z_maxs[i_slab - 1]) {
                        //pt also fits in overlap w/ next lower slab:
                        indices_by_slab[i_slab - 1].push_back(sel_pt);
                    }
                }
            }
        } else {
            //ROS_INFO("pt %d: Nan!", ipt);  //got LOTS of NaN's; (pts > 260,000); don't know why
        }
    } // done sorting points by slabs
    // find the slab with the most points:
    i_best_slab = 0;
    int npts_islab = indices_by_slab[i_best_slab].size();
    int npts_best_slab = npts_islab;


   // ROS_INFO("npts slab %d = %d", i_best_slab, npts_best_slab);
    for (int islab = 1; islab < nslabs; islab++) {
        npts_islab = indices_by_slab[islab].size();
        //ROS_INFO("npts slab %d = %d", islab, npts_islab);
        if (npts_islab > npts_best_slab) {
            npts_best_slab = npts_islab;
            i_best_slab = islab;
        }
    }
    //ROS_INFO("best slab num = %d with %d pts", i_best_slab, npts_best_slab);
}

// fnc to find floor; as part of process, sorts all points from an input point cloud into horizontal slabs that should bound the floor
// provide an input cloud; prototype sets default values for z-search range and resolution;
// may override default z_min, z_max and dz_slab if desired
// results are in member vars, including: indices_by_slab_, i_best_slab_, z_mins_, z_maxs_
// ALSO, put selected indices from pointcloud into indices_floor

bool Object_finder::find_floor(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> &indices_floor, double dz_slab, double z_search_min, double z_search_max) {
    //plan: first sort points by slabs; put results in member vars
    sort_pts_by_horiz_planes(inputCloudWrtBase, z_search_min, z_search_max, dz_slab, indices_by_slab_, i_best_slab_, z_mins_, z_maxs_);

    // find most populous slab BELOW FLOOR_SEARCH_MAX_Z
    int nslabs = indices_by_slab_.size();
    int islab_floor = 0;
    int npts_floor = 0;
    vector<int> indices_slab_i; // = indices_by_slab_[i_best_slab_]; // select points from most populous slab    
    // search up from bottom, stopping at max credible floor height (must be below table height!)
    for (int islab = 0; islab < nslabs; islab++) {
        if (z_maxs_[islab] > FLOOR_SEARCH_MAX_Z) {
            ROS_INFO("stopping floor search at islab = %d", islab);
            break; //consider only slabs below floor search height
        }
        indices_slab_i = indices_by_slab_[islab]; // indices of slab at this height
        if (indices_slab_i.size() > npts_floor) { // how many points in this slab?  more than best so far?
            npts_floor = indices_slab_i.size();
            islab_floor = islab;
        }
    } //...and the winner is...
    // at this point, islab_floor has the most points within floor search range;
    indices_slab_i = indices_by_slab_[islab_floor];

    //take the most populated slab and subdivide it:


    //vector<int> indices_slab_i = indices_by_slab_[i_best_slab_]; // select points from most populous slab
    //int npts_islab = indices_slab_i.size();
    //int npts_best_slab = 0;
    double z_slabs_min = z_mins_[islab_floor]; //lower and upper z-bounds of most populous slab
    double z_slabs_max = z_maxs_[islab_floor];
    ROS_INFO("coarse find: floor is between z= %f and %f; subdivide:", z_slabs_min, z_slabs_max);
    double dz_slab_fine = DZ_SLAB_1CM; //this is likely too precise for real data...1cm slabs


    // for points within chosen thick slab, slice into thinner slabs:
    //put results here:
    int i_best_slab = 0;
    vector<float> z_mins;
    vector<float> z_maxs;
    vector< vector<int> >indices_by_slab;
    sort_pts_by_horiz_planes(inputCloudWrtBase, indices_slab_i, z_slabs_min, z_slabs_max, dz_slab_fine, indices_by_slab, i_best_slab, z_mins, z_maxs);
    nslabs = indices_by_slab.size();
    indices_floor = indices_by_slab[i_best_slab];
    npts_floor = indices_floor.size();
    double z_floor_min = z_mins[i_best_slab];
    double z_floor_max = z_maxs[i_best_slab];
    ROS_INFO("fine search: %d floor points between z = %f and %f ", npts_floor, z_floor_min, z_floor_max);
    // could fit a plane to these points...
    z_floor_ = (z_floor_min + z_floor_max) / 2.0;
    if (npts_floor > 0) // could put a higher threshold on this...
        return true;
    else
        return false;

}

//ASSUMES have already sorted points into thick slabs;
// find best slab for horizontal surface within specified search range

int Object_finder::find_surface(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> &indices_surface,
        double dz_slab, double z_search_min, double z_search_max, double &z_surface) {
    //plan: first sort points by slabs; put results in member vars   
    // find most populous slab within search range
    int nslabs = indices_by_slab_.size(); // this is ALL of the slabs
    int islab_surface = 0;
    int npts_surface = 0;
    vector<int> indices_slab_i; // = indices_by_slab_[i_best_slab_]; // select points from most populous slab    
    // search up from bottom, considering pts between min and max credible table height 
    for (int islab = 0; islab < nslabs; islab++) {
        if ((z_maxs_[islab] > z_search_min) && (z_maxs_[islab] < z_search_max)) { //here is within table height search range
            indices_slab_i = indices_by_slab_[islab]; // indices of slab at this height
            if (indices_slab_i.size() > npts_surface) { // how many points in this slab?  more than best so far?
                npts_surface = indices_slab_i.size();
                islab_surface = islab;
            }
        }
    }//...and the winner is...
    // at this point, islab_table has the most points within table search range;
    indices_slab_i = indices_by_slab_[islab_surface];

    //take this most populated slab and subdivide it:
    double z_slab_min = z_mins_[islab_surface]; //lower and upper z-bounds of most populous slab
    double z_slab_max = z_maxs_[islab_surface];
    ROS_INFO("coarse find: surface w/ %d pts is between z= %f and %f; subdivide:", npts_surface, z_slab_min, z_slab_max);

    // for points within chosen thick slab, slice into thinner slabs:
    //put results here:
    int i_best_slab = 0;
    vector<float> z_mins;
    vector<float> z_maxs;
    vector< vector<int> >indices_by_slab;
    sort_pts_by_horiz_planes(inputCloudWrtBase, indices_slab_i, z_slab_min, z_slab_max, dz_slab, indices_by_slab, i_best_slab, z_mins, z_maxs);
    nslabs = indices_by_slab.size();
    indices_surface = indices_by_slab[i_best_slab];
    npts_surface = indices_surface.size();
    double z_surface_min = z_mins[i_best_slab];
    double z_surface_max = z_maxs[i_best_slab];
    ROS_INFO("fine search: %d surface points between z = %f and %f ", npts_surface, z_surface_min, z_surface_max);
    z_surface = (z_surface_min + z_surface_max) / 2.0;
    z_top_of_object_ = z_surface; // return the value, and store it as internal var as well

    // could fit a plane to these points...
    return npts_surface;
}

//alt version:  accepts index list input
int Object_finder::find_surface(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> input_indices, vector<int> &indices_surface,
        double dz_slab, double z_search_min, double z_search_max, double &z_surface) {
    //plan: first sort points by slabs; put results in member vars   
    // find most populous slab within search range
    
    /*
    int nslabs = indices_by_slab_.size(); // this is ALL of the slabs
    int islab_surface = 0;
    int npts_surface = 0;
    vector<int> indices_slab_i; // = indices_by_slab_[i_best_slab_]; // select points from most populous slab    
    // search up from bottom, considering pts between min and max credible table height 
    for (int islab = 0; islab < nslabs; islab++) {
        if ((z_maxs_[islab] > z_search_min) && (z_maxs_[islab] < z_search_max)) { //here is within table height search range
            indices_slab_i = indices_by_slab_[islab]; // indices of slab at this height
            if (indices_slab_i.size() > npts_surface) { // how many points in this slab?  more than best so far?
                npts_surface = indices_slab_i.size();
                islab_surface = islab;
            }
        }
    }//...and the winner is...
    // at this point, islab_table has the most points within table search range;
    indices_slab_i = indices_by_slab_[islab_surface];
    */
    //subdivide by thin layers:
    //double z_slab_min = z_mins_[islab_surface]; //lower and upper z-bounds of most populous slab
    //double z_slab_max = z_maxs_[islab_surface];
    //ROS_INFO("coarse find: surface w/ %d pts is between z= %f and %f; subdivide:", npts_surface, z_slab_min, z_slab_max);

    // for points within chosen thick slab, slice into thinner slabs:
    //put results here:
    int i_best_slab = 0;
    vector<float> z_mins;
    vector<float> z_maxs;
    vector< vector<int> >indices_by_slab;
    vector<int> indices_slab_i; // 

    sort_pts_by_horiz_planes(inputCloudWrtBase, input_indices, z_search_min, z_search_max, dz_slab, indices_by_slab, i_best_slab, z_mins, z_maxs);
    int nslabs = indices_by_slab.size();
    indices_surface = indices_by_slab[i_best_slab];
    int npts_surface = indices_surface.size();
    double z_surface_min = z_mins[i_best_slab];
    double z_surface_max = z_maxs[i_best_slab];
    ROS_INFO("fine search: %d surface points between z = %f and %f ", npts_surface, z_surface_min, z_surface_max);
    z_surface = (z_surface_min + z_surface_max) / 2.0;
    z_top_of_object_ = z_surface; // return the value, and store it as internal var as well

    // could fit a plane to these points...
    return npts_surface;
}

//has default values for dz_slab, z_search_min and z_search_max
// return number of pts in table slab; fill in indices_table and z_table

int Object_finder::find_table(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> &indices_table, double &z_table,
        double dz_slab, double z_search_min, double z_search_max) {
    int npts_table = find_surface(inputCloudWrtBase, indices_table, dz_slab, z_search_min, z_search_max, z_table_);
    ROS_INFO("presumed table slab has %d points at height %f", npts_table, z_table_);
    z_table = z_table_; //return table ht, and also store it as internal var
    return npts_table;
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

