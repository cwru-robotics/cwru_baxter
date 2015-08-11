//start of class for associating grasp-frame poses relative to a model frame
// to be populated with (many) special-purpose functions
// given a model frame, return a vector of candidate grasp poses, as a vector of Eigen::Affine3d transforms
// of grasp frame w/rt model frame


class Object_grasp_frame_computer {
public:
    Object_grasp_frame_computer(); //constructor
    // since multiple grasp frames may exist, a vector of transforms is filled;
    // these correspond to options for yale-hand grasp frames w/rt Baxter base frame
    void find_coke_can_frame_grasp_frames_from_above(std::vector<Eigen::Affine3d> &grasp_xforms);
    // add lots more special-purpose fncs here...

private:
   
}; //end of class definition; later, move this to a header file

Object_grasp_frame_computer::Object_grasp_frame_computer() { // constructor
    // do initializations here
}

//user results in grasp_xforms as follows:
    // A_grasp_wrt_model = sup{model}_A_sub{grasp}
    // A_grasp_wrt_base = sup{base}_A_sub{grasp}
    // sup{base}_A_sub{grasp} = sup{base}_A_sub{model}*sup{model}_A_sub{grasp}
    // affine_grasp_wrt_base = affine_model_frame_wrt_base*grasp_frame_wrt_model_frame;
void Object_grasp_frame_computer::find_coke_can_frame_grasp_frames_from_above(std::vector<Eigen::Affine3d> &grasp_xforms) {
    
    //coke can dimensions, per Kinect data
    const double RADIUS = 0.03; //estimated from ruler tool...example to fit a cylinder of this radius to data
    const double H_CYLINDER = 0.12; // estimated height of cylinder
    const double GRASP_V_OFFSET = 0.1; // TUNE THIS: appropriate vertical offset for can grasp from above
    Eigen::Affine3d grasp_frame_wrt_model_frame;
    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    
    Eigen::Vector3d grasp_origin_wrt_model_origin;
    grasp_origin_wrt_model_origin(0) = 0.0;
    grasp_origin_wrt_model_origin(1) = 0.0;
    grasp_origin_wrt_model_origin(2) = H_CYLINDER + GRASP_V_OFFSET;
    
    grasp_frame_wrt_model_frame.linear() = R; // align gripper frame w/ model frame...many more options exist w/ rotation of gripper about gripper-z axis
    grasp_frame_wrt_model_frame.translation() = grasp_origin_wrt_model_origin; // vertical offset for grasp from above    

    // could put in lots more options, but we'll just return this one option for now:
    grasp_xforms.clear();
    grasp_xforms.push_back(grasp_frame_wrt_model_frame);

}



