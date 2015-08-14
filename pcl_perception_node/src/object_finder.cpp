const double TABLE_VIEW_X_MIN = 0.35; //0.35; //0.3 picks up some of e-stop box
const double TABLE_VIEW_X_MAX = 1;
const double TABLE_VIEW_Y_MIN = -0.4;
const double TABLE_VIEW_Y_MAX = 0.4;

//search ranges for floor and table, from base-frame coords
const double FLOOR_SEARCH_MIN_Z = -1.5;
const double FLOOR_SEARCH_MAX_Z = -0.5;
const double DZ_SLAB_5CM = 0.05;
const double DZ_SLAB_1CM = 0.01;

const double TABLE_SEARCH_MIN_Z = -0.5;
const double TABLE_SEARCH_MAX_Z = 0.0;


const double R_GAZEBO_BEER = 0.055; //estimated from ruler tool...example to fit a cylinder of this radius to data
const double H_GAZEBO_BEER = 0.23; // estimated height of cylinder

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
    // add lots more special-purpose fncs here...
    Eigen::Affine3d find_gazebo_beer_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, double table_height);
    ///given a pointer to a point cloud and a list if inidices of interest, find list of indices for which the points
    // lie within a prescribed x and y window
    void xy_window(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> input_indices, vector<int> &output_indices, 
      double x_min=TABLE_VIEW_X_MIN, double x_max=TABLE_VIEW_X_MAX, double y_min=TABLE_VIEW_Y_MIN, double y_max=TABLE_VIEW_Y_MAX); 
   void sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, double z_search_min, double z_search_max, double dz_slab, 
            std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs);
    void sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> indices, double z_search_min, double z_search_max, double dz_slab, 
            std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs);
    bool find_floor(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, 
        double dz_slab = DZ_SLAB_5CM, double z_search_min=FLOOR_SEARCH_MIN_Z, double z_search_max=FLOOR_SEARCH_MAX_Z);
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

    std::vector<vector<int> > indices_by_slab_;
    int i_best_slab_;
    vector<float> z_mins_; 
    vector<float> z_maxs_;
    Eigen::Vector4f plane_params_; 
    double z_floor_;
    
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
    ROS_INFO("box filter of %d pts",npts);
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
    ROS_INFO("window filter of %d pts",npts);
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

Eigen::Affine3d Object_finder::find_gazebo_beer_can_frame(PointCloud<pcl::PointXYZ>::Ptr pt_cloud_wrt_base, 
        double table_height=default_cafe_table_z_height_wrt_baxter_base_frame) {
    //coke can dimensions, per Kinect data
    const double RADIUS = R_GAZEBO_BEER; //estimated from ruler tool...example to fit a cylinder of this radius to data
    const double H_CYLINDER = H_GAZEBO_BEER; // estimated height of cylinder
    const double GRASP_V_OFFSET = 0.1; // TUNE THIS: appropriate vertical offset for can grasp from above
    // look for points +/- 1cm from expected can height
    // gazebo beer-can top is at z= 0.075
    double z_min = table_height + H_CYLINDER - 0.01; 
    double z_max = table_height + H_CYLINDER + 0.01;
    ROS_INFO("looking for gazebo beer can");
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

void Object_finder::sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, double z_search_min, double z_search_max, double dz_slab, 
            std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs) {

    int npts = inputCloudWrtBase->width * inputCloudWrtBase->height;
    Eigen::Vector3f pt;
    // sort points from z_min to z_max in slabs of thickness dz_slab
    // allow overlap of z_overlap to avoid boundary issues
    double z_overlap = 0.02; // choose this much overlap
    int nslabs =  1+(z_search_max-z_search_min+z_overlap)/dz_slab;
    ROS_INFO("sorting points into %d slabs",nslabs);
    //vector<float> z_mins;
    //vector<float> z_maxs;
    z_mins.resize(nslabs);
    z_maxs.resize(nslabs);
    z_mins[0] = z_search_min;
    z_maxs[0] = z_search_min+dz_slab+z_overlap;
    for (int i=1;i<nslabs;i++) {
        z_mins[i] = z_mins[i-1]+dz_slab;
        z_maxs[i] = z_mins[i] + dz_slab + z_overlap;       
    }
    //classify each pt by slab membership
    //vector<vector<int> > indices_by_slab; 
    indices_by_slab.clear();
    indices_by_slab.resize(nslabs);
    ROS_INFO("clearing out vector of vectors...");
    for (int i=0;i<nslabs;i++) indices_by_slab[i].clear();
    double z_height;
    int i_slab;
    // given z-height, nominal slab number = floor((z_height-z_search_min)/dz_slab)
    // but also check above and below overlaps
   for (int ipt=0;ipt<npts;ipt++) {
        pt = inputCloudWrtBase->points[ipt].getVector3fMap();
        //ROS_INFO(" pt %d: %f, %f, %f",ipt,pt(0),pt(1),pt(2));
        z_height = pt(2);
        if (z_height==z_height) { // watch out for nan?
        i_slab = floor((z_height-z_search_min)/dz_slab); //nom slab membership--but check above and below as well
        indices_by_slab[i_slab].push_back(ipt);
        if (i_slab< nslabs-2) {  // not the top slab--so look above
             if (z_height>z_mins[i_slab+1]) {
            //pt also fits in overlap w/ next higher slab:
            indices_by_slab[i_slab+1].push_back(ipt);
             }
        }
        //check below:
        if (i_slab>0){ // not the bottom slab
            if (z_height<z_maxs[i_slab-1]) {
                //pt also fits in overlap w/ next lower slab:
                indices_by_slab[i_slab-1].push_back(ipt);
            }
        }   
        }
        else  {
            //ROS_INFO("pt %d: Nan!", ipt);  //got LOTS of NaN's; (pts > 260,000); don't know why
        }
   } // done sorting points by slabs
    // find the slab with the most points:
    i_best_slab=0;
    int npts_islab = indices_by_slab[i_best_slab].size();
    int npts_best_slab = npts_islab;
    

    ROS_INFO("npts slab %d = %d",i_best_slab,npts_best_slab);
    for (int islab=1;islab<nslabs;islab++) {
        npts_islab = indices_by_slab[islab].size();
        ROS_INFO("npts slab %d = %d",islab,npts_islab);
        if (npts_islab>npts_best_slab) {
           npts_best_slab = npts_islab; 
           i_best_slab = islab;
        }
    }
    ROS_INFO("best slab num = %d with %d pts",i_best_slab,npts_best_slab);
}

// variant--operate only on listed points from pointcloud
void Object_finder::sort_pts_by_horiz_planes(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, vector<int> indices, double z_search_min, double z_search_max, double dz_slab, 
            std::vector<vector<int> > &indices_by_slab, int &i_best_slab, vector<float> &z_mins, vector<float> &z_maxs) {
    int npts = indices.size(); //inputCloudWrtBase->width * inputCloudWrtBase->height;
    Eigen::Vector3f pt;
    // sort points from z_min to z_max in slabs of thickness dz_slab
    // allow overlap of z_overlap to avoid boundary issues
    double z_overlap = 0.001; // choose this much overlap
    int nslabs =  1+(z_search_max-z_search_min+z_overlap)/dz_slab;
    ROS_INFO("sorting points into %d slabs",nslabs);
    //vector<float> z_mins;
    //vector<float> z_maxs;
    z_mins.resize(nslabs);
    z_maxs.resize(nslabs);
    z_mins[0] = z_search_min;
    z_maxs[0] = z_search_min+dz_slab+z_overlap;
    for (int i=1;i<nslabs;i++) {
        z_mins[i] = z_mins[i-1]+dz_slab;
        z_maxs[i] = z_mins[i] + dz_slab + z_overlap;       
    }
    //classify each pt by slab membership
    //vector<vector<int> > indices_by_slab; 
    indices_by_slab.clear();
    indices_by_slab.resize(nslabs);
    ROS_INFO("clearing out vector of vectors...");
    for (int i=0;i<nslabs;i++) indices_by_slab[i].clear();
    double z_height;
    int i_slab;
    // given z-height, nominal slab number = floor((z_height-z_search_min)/dz_slab)
    // but also check above and below overlaps
    int sel_pt;
   for (int ipt=0;ipt<npts;ipt++) {
       sel_pt = indices[ipt];
        pt = inputCloudWrtBase->points[sel_pt].getVector3fMap();
        //ROS_INFO(" pt %d: %f, %f, %f",ipt,pt(0),pt(1),pt(2));
        z_height = pt(2);
        if (z_height==z_height) { // watch out for nan?
        i_slab = floor((z_height-z_search_min)/dz_slab); //nom slab membership--but check above and below as well
        indices_by_slab[i_slab].push_back(sel_pt);
        if (i_slab< nslabs-2) {  // not the top slab--so look above
             if (z_height>z_mins[i_slab+1]) {
            //pt also fits in overlap w/ next higher slab:
            indices_by_slab[i_slab+1].push_back(sel_pt);
             }
        }
        //check below:
        if (i_slab>0){ // not the bottom slab
            if (z_height<z_maxs[i_slab-1]) {
                //pt also fits in overlap w/ next lower slab:
                indices_by_slab[i_slab-1].push_back(sel_pt);
            }
        }   
        }
        else  {
            //ROS_INFO("pt %d: Nan!", ipt);  //got LOTS of NaN's; (pts > 260,000); don't know why
        }
   } // done sorting points by slabs
    // find the slab with the most points:
    i_best_slab=0;
    int npts_islab = indices_by_slab[i_best_slab].size();
    int npts_best_slab = npts_islab;
    

    ROS_INFO("npts slab %d = %d",i_best_slab,npts_best_slab);
    for (int islab=1;islab<nslabs;islab++) {
        npts_islab = indices_by_slab[islab].size();
        ROS_INFO("npts slab %d = %d",islab,npts_islab);
        if (npts_islab>npts_best_slab) {
           npts_best_slab = npts_islab; 
           i_best_slab = islab;
        }
    }
    ROS_INFO("best slab num = %d with %d pts",i_best_slab,npts_best_slab);
}    

// fnc to find floor; as part of process, sorts all points from an input point cloud into horizontal slabs that should bound the floor
// provide an input cloud; may override default z_min, z_max and dz_slab; returns indices of floor points and 
bool Object_finder::find_floor(PointCloud<pcl::PointXYZ>::Ptr inputCloudWrtBase, double dz_slab, double z_search_min, double z_search_max) {
    //plan: first sort points by slabs; 
    sort_pts_by_horiz_planes(inputCloudWrtBase, z_search_min, z_search_max, dz_slab, indices_by_slab_, i_best_slab_, z_mins_, z_maxs_);
    //take the most populated slab in subdivide it:
    int i_best_slab=0;
    vector<int> indices_slab_i;
    vector<float> z_mins;
    vector<float> z_maxs;
    vector< vector<int> >indices_by_slab;
    indices_slab_i = indices_by_slab_[i_best_slab_];
    int npts_islab = indices_slab_i.size();
    int npts_best_slab = 0;    
    double z_slabs_min = z_mins_[i_best_slab_];
    double z_slabs_max = z_maxs_[i_best_slab_];
    //double dz_slab = DZ_SLAB_1CM;
                    
    // for points within chosen thick slab, slice into thinner slabs:
    sort_pts_by_horiz_planes(inputCloudWrtBase,indices_slab_i, z_slabs_min, z_slabs_max, dz_slab, indices_by_slab, i_best_slab, z_mins,z_maxs);
    int nslabs = indices_by_slab.size();    
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

