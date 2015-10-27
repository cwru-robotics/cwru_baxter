// cwru_pcl_utils: a  ROS library to illustrate use of PCL, including some handy utility functions
//

#include <cwru_pcl_utils/cwru_pcl_utils.h>
//uses initializer list for member vars
CwruPclUtils::CwruPclUtils(ros::NodeHandle* nodehandle):nh_(*nodehandle),pclKinect_ptr_(new PointCloud<pcl::PointXYZ>),
pclTransformed_ptr_(new PointCloud<pcl::PointXYZ>) {
    initializeSubscribers(); 
    got_kinect_cloud_=false;
}

void CwruPclUtils::fit_points_to_plane(Eigen::MatrixXd points_mat,Eigen::Vector3d &plane_normal, double &plane_dist) {
    //ROS_INFO("starting identification of plane from data: ");
    // first compute the centroid of the data:
    Eigen::Vector3d centroid;
    // here's a handy way to initialize data to all zeros; more variants exist
    centroid = Eigen::MatrixXd::Zero(3,1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    //add all the points together:
    int npts = points_mat.cols(); // number of points = number of columns in matrix; check the size
    //cout<<"matrix has ncols = "<<npts<<endl;
    for (int ipt =0;ipt<npts;ipt++) {
	centroid+= points_mat.col(ipt); //add all the column vectors together
    }
    centroid/=npts; //divide by the number of points to get the centroid
    //cout<<"centroid: "<<centroid.transpose()<<endl;
    
    
    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXd points_offset_mat = points_mat;
    for (int ipt =0;ipt<npts;ipt++) {
        points_offset_mat.col(ipt)  = points_offset_mat.col(ipt)-centroid;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3d CoVar;
    CoVar = points_offset_mat*(points_offset_mat.transpose());  //3xN matrix times Nx3 matrix is 3x3
    //cout<<"covariance: "<<endl;
    //cout<<CoVar<<endl;
    
    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3d> es3d(CoVar);
    
    Eigen::VectorXd evals; //we'll extract the eigenvalues to here
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

    evals= es3d.eigenvalues().real(); // grab just the real parts
    //cout<<"real parts of evals: "<<evals.transpose()<<endl;

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves
    
    double min_lambda = evals[0]; //initialize the hunt for min eval
    Eigen::Vector3cd complex_vec; // here is a 3x1 vector of double-precision, complex numbers

    complex_vec=es3d.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    plane_normal = complex_vec.real();  //strip off the real part
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec=1;ivec<3;ivec++) {
        lambda_test = evals[ivec];
    	if (lambda_test<min_lambda) {
		min_lambda =lambda_test;
                i_normal= ivec; //this index is closer to index of min eval
		plane_normal = es3d.eigenvectors().col(ivec).real();
        }
    }
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    //cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    //cout<<"corresponding evec (est plane normal): "<<est_plane_normal.transpose()<<endl;
    //cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    plane_dist = plane_normal.dot(centroid);
    //cout<<"est plane distance from origin = "<<est_dist<<endl;
    //cout<<"correct answer is: "<<dist<<endl;
    //cout<<endl<<endl;    
    
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
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void CwruPclUtils::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    
    pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &CwruPclUtils::kinectCB,this);  
    // add more subscribers here, as needed
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
        ROS_INFO("kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_->width, (int) pclKinect_ptr_->height);
        got_kinect_cloud_=true; //cue to "main" that callback received and saved a pointcloud        
    }
    //pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
    //ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd",(int) g_pclKinect->points.size ()); 
}
