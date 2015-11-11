// cwru_pcl_utils: a  ROS library to illustrate use of PCL, including some handy utility functions
//

#include <cwru_pcl_utils/cwru_pcl_utils.h>
//uses initializer list for member vars

CwruPclUtils::CwruPclUtils(ros::NodeHandle* nodehandle) : nh_(*nodehandle), pclKinect_ptr_(new PointCloud<pcl::PointXYZ>),
        pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>),
pclTransformed_ptr_(new PointCloud<pcl::PointXYZ>), pclSelectedPoints_ptr_(new PointCloud<pcl::PointXYZ>),
pclTransformedSelectedPoints_ptr_(new PointCloud<pcl::PointXYZ>),pclGenPurposeCloud_ptr_(new PointCloud<pcl::PointXYZ>) {
    initializeSubscribers();
    initializePublishers();
    got_kinect_cloud_ = false;
    got_selected_points_ = false;
}




void CwruPclUtils::fit_points_to_plane(Eigen::MatrixXf points_mat, Eigen::Vector3f &plane_normal, double &plane_dist) {
    //ROS_INFO("starting identification of plane from data: ");
    int npts = points_mat.cols(); // number of points = number of columns in matrix; check the size
    
    // first compute the centroid of the data:
    //Eigen::Vector3f centroid; // make this member var, centroid_
    centroid_ = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    
    //centroid = compute_centroid(points_mat);
     for (int ipt = 0; ipt < npts; ipt++) {
        centroid_ += points_mat.col(ipt); //add all the column vectors together
    }
    centroid_ /= npts; //divide by the number of points to get the centroid    
    cout<<"centroid: "<<centroid_.transpose()<<endl;


    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXf points_offset_mat = points_mat;
    for (int ipt = 0; ipt < npts; ipt++) {
        points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose()); //3xN matrix times Nx3 matrix is 3x3
    //cout<<"covariance: "<<endl;
    //cout<<CoVar<<endl;

    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

    Eigen::VectorXf evals; //we'll extract the eigenvalues to here
    //cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    //cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    //cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    //cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
    //cout << "The matrix of eigenvectors, V, is:" << endl;
    //cout<< es3d.eigenvectors() << endl << endl;
    //cout<< "(these should be real-valued vectors)"<<endl;
    // in general, the eigenvalues/eigenvectors can be complex numbers
    //however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
    // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

    evals = es3f.eigenvalues().real(); // grab just the real parts
    //cout<<"real parts of evals: "<<evals.transpose()<<endl;

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves

    double min_lambda = evals[0]; //initialize the hunt for min eval
    double max_lambda = evals[0]; // and for max eval
    //Eigen::Vector3cf complex_vec; // here is a 3x1 vector of double-precision, complex numbers
    //Eigen::Vector3f evec0, evec1, evec2; //, major_axis; 
    //evec0 = es3f.eigenvectors().col(0).real();
    //evec1 = es3f.eigenvectors().col(1).real();
    //evec2 = es3f.eigenvectors().col(2).real();  
    
    
    //((pt-centroid)*evec)*2 = evec'*points_offset_mat'*points_offset_mat*evec = 
    // = evec'*CoVar*evec = evec'*lambda*evec = lambda
    // min lambda is ideally zero for evec= plane_normal, since points_offset_mat*plane_normal~= 0
    // max lambda is associated with direction of major axis
    
    //sort the evals:
    
    //complex_vec = es3f.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    plane_normal = es3f.eigenvectors().col(0).real(); //complex_vec.real(); //strip off the real part
    major_axis_ = es3f.eigenvectors().col(0).real(); // starting assumptions
    
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal = 0;
    int i_major_axis=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec = 1; ivec < 3; ivec++) {
        lambda_test = evals[ivec];
        if (lambda_test < min_lambda) {
            min_lambda = lambda_test;
            i_normal = ivec; //this index is closer to index of min eval
            plane_normal = es3f.eigenvectors().col(i_normal).real();
        }
        if (lambda_test > max_lambda) {
            max_lambda = lambda_test;
            i_major_axis = ivec; //this index is closer to index of min eval
            major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
        }        
    }
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    cout<<"corresponding evec (est plane normal): "<<plane_normal.transpose()<<endl;
    cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
    cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;    
    
    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    plane_dist = plane_normal.dot(centroid_);
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;    

}

//get pts from cloud, pack the points into an Eigen::MatrixXf, then use above
// fit_points_to_plane fnc

void CwruPclUtils::fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::Vector3f &plane_normal, double &plane_dist) {
    Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    //populate points_mat from cloud data;

    int npts = input_cloud_ptr->points.size();
    points_mat.resize(3, npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        cloud_pt = input_cloud_ptr->points[i].getVector3fMap();
        points_mat.col(i) = cloud_pt;
    }
    fit_points_to_plane(points_mat, plane_normal, plane_dist);

}

//compute and return the centroid of a pointCloud
Eigen::Vector3f  CwruPclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr) {
    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;   
    int npts = input_cloud_ptr->points.size();    
    centroid<<0,0,0;
    //add all the points together:

    for (int ipt = 0; ipt < npts; ipt++) {
        cloud_pt = input_cloud_ptr->points[ipt].getVector3fMap();
        centroid += cloud_pt; //add all the column vectors together
    }
    centroid/= npts; //divide by the number of points to get the centroid
    return centroid;
}

// this fnc operates on transformed selected points

void CwruPclUtils::fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal, double &plane_dist) {
    fit_points_to_plane(pclTransformedSelectedPoints_ptr_, plane_normal, plane_dist);
    //Eigen::Vector3f centroid;
    cwru_msgs::PatchParams patch_params_msg;
    //compute the centroid; this is redundant w/ computation inside fit_points...oh well.
    // now the centroid computed by plane fit is stored in centroid_ member var
    //centroid = compute_centroid(pclTransformedSelectedPoints_ptr_);

    patch_params_msg.offset = plane_dist;
    patch_params_msg.centroid.resize(3);
    patch_params_msg.normal_vec.resize(3);
    for (int i=0;i<3;i++) {
        patch_params_msg.normal_vec[i]=plane_normal[i];
        patch_params_msg.centroid[i]= centroid_[i];
    }
    patch_params_msg.frame_id = "torso";
    patch_publisher_.publish(patch_params_msg);
}

Eigen::Affine3f CwruPclUtils::transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3f e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

/**here is a function that transforms a cloud of points into an alternative frame;
 * it assumes use of pclKinect_ptr_ from kinect sensor as input, to pclTransformed_ptr_ , the cloud in output frame
 * 
 * @param A [in] supply an Eigen::Affine3f, such that output_points = A*input_points
 */
void CwruPclUtils::transform_kinect_cloud(Eigen::Affine3f A) {
    transform_cloud(A, pclKinect_ptr_, pclTransformed_ptr_);
    /*
    pclTransformed_ptr_->header = pclKinect_ptr_->header;
    pclTransformed_ptr_->is_dense = pclKinect_ptr_->is_dense;
    pclTransformed_ptr_->width = pclKinect_ptr_->width;
    pclTransformed_ptr_->height = pclKinect_ptr_->height;
    int npts = pclKinect_ptr_->points.size();
    cout << "transforming npts = " << npts << endl;
    pclTransformed_ptr_->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        pclTransformed_ptr_->points[i].getVector3fMap() = A * pclKinect_ptr_->points[i].getVector3fMap(); 
    }    
     * */
}

void CwruPclUtils::transform_selected_points_cloud(Eigen::Affine3f A) {
    transform_cloud(A, pclSelectedPoints_ptr_, pclTransformedSelectedPoints_ptr_);
}

//    void get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud );

void CwruPclUtils::get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud ) {
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); //how many points to extract?
    outputCloud.header = pclTransformedSelectedPoints_ptr_->header;
    outputCloud.is_dense = pclTransformedSelectedPoints_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclTransformedSelectedPoints_ptr_->points[i].getVector3fMap();   
    }
}

//same as above, but for general-purpose cloud
void CwruPclUtils::get_gen_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> & outputCloud ) {
    int npts = pclGenPurposeCloud_ptr_->points.size(); //how many points to extract?
    outputCloud.header = pclGenPurposeCloud_ptr_->header;
    outputCloud.is_dense = pclGenPurposeCloud_ptr_->is_dense;
    outputCloud.width = npts;
    outputCloud.height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud.points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap();   
    }    
} 

//here is an example utility function.  It operates on clouds that are member variables, and it puts its result
// in the general-purpose cloud variable, which can be acquired by main(), if desired, using get_gen_purpose_cloud()

// The operation illustrated here is not all that useful.  It uses transformed, selected points,
// elevates the data by 5cm, and copies the result to the general-purpose cloud variable
void CwruPclUtils::example_pcl_operation() {
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); //number of points
    copy_cloud(pclTransformedSelectedPoints_ptr_,pclGenPurposeCloud_ptr_); //now have a copy of the selected points in gen-purpose object
    Eigen::Vector3f offset;
    offset<<0,0,0.05;
    for (int i = 0; i < npts; ++i) {
        pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;   
    }    
} 


//generic function to copy an input cloud to an output cloud
// provide pointers to the two clouds
//output cloud will get resized
void CwruPclUtils::copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = inputCloud->points.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
    }
}

//need to fix this to put proper frame_id in header

void CwruPclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr) {
    output_cloud_ptr->header = input_cloud_ptr->header;
    output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
    output_cloud_ptr->width = input_cloud_ptr->width;
    output_cloud_ptr->height = input_cloud_ptr->height;
    int npts = input_cloud_ptr->points.size();
    cout << "transforming npts = " << npts << endl;
    output_cloud_ptr->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
    }
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass

void CwruPclUtils::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");

    pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &CwruPclUtils::kinectCB, this);
    // add more subscribers here, as needed

    // subscribe to "selected_points", which is published by Rviz tool
    selected_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, &CwruPclUtils::selectCB, this);
}

//member helper function to set up publishers;

void CwruPclUtils::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("cwru_pcl_pointcloud", 1, true);
    patch_publisher_ = nh_.advertise<cwru_msgs::PatchParams>("pcl_patch_params", 1, true);
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

/**
 * callback fnc: receives transmissions of Kinect data; if got_kinect_cloud is false, copy current transmission to internal variable
 * @param cloud [in] messages received from Kinect
 */
void CwruPclUtils::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    //cout<<"callback from kinect pointcloud pub"<<endl;
    // convert/copy the cloud only if desired
    if (!got_kinect_cloud_) {
        pcl::fromROSMsg(*cloud, *pclKinect_ptr_);
        pcl::fromROSMsg(*cloud,*pclKinect_clr_ptr_);
        ROS_INFO("kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_->width, (int) pclKinect_ptr_->height);
        got_kinect_cloud_ = true; //cue to "main" that callback received and saved a pointcloud        
    }
    //pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
    //ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd",(int) g_pclKinect->points.size ()); 
}

// this callback wakes up when a new "selected Points" message arrives

void CwruPclUtils::selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::fromROSMsg(*cloud, *pclSelectedPoints_ptr_);
    ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", pclSelectedPoints_ptr_->width, pclSelectedPoints_ptr_->height);
    got_selected_points_ = true;
}