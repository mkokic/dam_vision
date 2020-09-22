#include "hough_circle/hough_circle.h"
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>

namespace ipa_hough_circle
{
// Constructor
    Hough::Hough() : it(nh), nh()
    {
        int circle_detected = 0;

        // Camera intrinsics
        cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

        // distortion coefficients
        distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

        haveCamInfo = false;
        circle_detected = false;

        do_circle_detection = false;
        circle_detection_finished = false;

        f = boost::bind(&Hough::callback,this, _1, _2);
    	  server.setCallback(f);

        pub = it.advertise("/houghcircle", 1);  //Filtered image publisher
        sub = it.subscribe("/camera/rgb/image_raw", 1,
                        &Hough::imageCallback,this);
        bin_pub = it.advertise("/houghcircle_binary",1);

        vector_pub = nh.advertise<geometry_msgs::Vector3>("circlepos",10);
        pub_marker_ = nh.advertise<visualization_msgs::Marker>("cam_fov",10);

        caminfo_sub = nh.subscribe("/camera/rgb/camera_info", 1,
                        &Hough::camInfoCallback, this);

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",10);

        service = nh.advertiseService("camera_calibration", &Hough::srv_cam_pose,this);

        pub_rim = nh.advertise<hough_circle::my_msg>("rim_points", 100);

        ROS_INFO("Hough circle detection ready, Initialization complete");
        }

        bool Hough::srv_cam_pose(std_srvs::SetBool::Request &req,
                                 std_srvs::SetBool::Response &res)
        {
            if (req.data == true) {
                do_circle_detection = true;
                res.success = true;
                res.message = "Service called successfully";
                return true;
            }
            else
            {
                res.success = false;
                res.message = "Failed to call service ";
                do_circle_detection = false;
                iLastX = -1;
                iLastY = -1;
            }
        }

    // Doing the actual contour detection
    void Hough::houghDetection(const cv::Mat& src_blur, const cv::Mat& src_display, int cannyThreshold, int accumulatorThreshold, int counter)
    {
        display = src_display.clone();

        // Give it some time to start
        if (counter > 100)
        {
            //MIA
            std::vector<cv::Vec4i> hierarchy;
            std::vector<std::vector<cv::Point>> contours;

            // Find contours
            cv::findContours( src_blur, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );
            for(size_t i = 0; i < contours.size(); i++)
            {
                size_t count = contours[i].size();
                if( count < 6 )
                   continue;
                cv::Mat pointsf;
                cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
                cv::RotatedRect box = fitEllipse(pointsf);
                if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
                   continue;

                // Making sure it looks like a rim
                if( (box.size.width * box.size.height) < 40000 && (box.size.width * box.size.height) > 3600)
                {
                    if (iLastX == -1)
                    {
                        drawContours( display, contours, i, cv::Scalar(0,0,255), 2, 8, hierarchy, 0, cv::Point() );
                        iLastX = box.center.x;
                        iLastY = box.center.y;

                        rim_array.points.clear();
                        geometry_msgs::Point point;
                        point.x = iLastX;
                        point.y = iLastY;
                        rim_array.points.push_back(point);
                    }
                    if (sqrt((box.center.x - iLastX) * (box.center.x - iLastX) + (box.center.y - iLastY) * (box.center.y - iLastY)) < 50.0)
                    {
                        drawContours( display, contours, i, cv::Scalar(0,0,255), 2, 8, hierarchy, 0, cv::Point() );
                        iLastX = box.center.x;
                        iLastY = box.center.y;

                        rim_array.points.clear();
                        geometry_msgs::Point point;
                        point.x = iLastX;
                        point.y = iLastY;
                        rim_array.points.push_back(point);
                    }
                }
            }
        }
    }

    double Hough::positionAverage(std::vector<double> v)
    {
        if(!debug_mode)
        {
            sort(v.begin(), v.end());
            if (v.size() % 2 == 0)
            {
            average_y_points = (v[v.size()/2.0 - 1.0] + v[v.size()/2.0]) / 2.0;
            std::cout << std::endl << "Median = "
            << average_y_points
            << std::endl;
            circle_detection_finished = true;
            return average_y_points;

            }
            else
            {
            average_y_points = v[v.size()/2];
            std::cout << std::endl << "Median = " << v[v.size()/2]
            << std::endl;
            circle_detection_finished = true;
            return average_y_points;

            }

        }
        else
        {
          //do nothing
          circle_detection_finished = false;
        }


    }


    void Hough::callback(hough_circle::ThresholdConfig &config, uint32_t level)
    {
    cannyThresholdInitialValue=config.canny_thres;
    accumulatorThresholdInitialValue= config.accumulator_thres;
    max_radius = config.max_radius;
    min_radius = config.min_radius;
    binary_thres = config.binary_thres;
    debug_mode = config.debugging;
    image_stream = config.image_stream;
    }

    void Hough::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
    // Read the image
	cv_bridge::CvImagePtr src_; //OpenCV full Image variable
	try
	    {
		src_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //Conversion
		src = src_->image;  //Assign just the image information to a Mat variable

	    }
	catch (cv_bridge::Exception& e)
	    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	    }
         int key = 0;
         run(); // proceed to the run function
    }

    void Hough::run(){
        if (do_circle_detection)
        {
           if(!src.empty())
            {

            int iLastX = -1;
            int iLastY = -1;

            // //Converting image from BGR to HSV color space.
            cv::Mat hsv;
            cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

            cv::Mat hue_image;
            cv::inRange(hsv, cv::Scalar(0, 0, 212), cv::Scalar(131, 255, 255), hue_image);
            cv::GaussianBlur(hue_image, hue_image, cv::Size(9, 9), 2, 2);

            //declare and initialize both parameters that are subjects to change
            int cannyThreshold = cannyThresholdInitialValue;
            int accumulatorThreshold = accumulatorThresholdInitialValue;

            // ROS_INFO("Counter num:");
            // std::cout << ct_ << std::endl;
            ct_ = ct_ + 1;

            //runs the detection, and update the display with messages
            houghDetection(hue_image, src, cannyThreshold, accumulatorThreshold, ct_);

            // contourDetection(src_blur);
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();

            pub.publish(msg);

            pub_rim.publish(rim_array);
            }
        }

    }

    void Hough::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
    sensor_msgs::CameraInfo camerainfo = *msg;
    getMarker(camerainfo);

    if (haveCamInfo) {
        return;
    }
    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}))
    {
        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
         }

    }

    void Hough::getMarker(sensor_msgs::CameraInfo camerainfo)
    {
        bool show_marker;
        double min_distance, max_distance;
        std_msgs::Header header;

        image_geometry::PinholeCameraModel pinmodel;
        pinmodel.fromCameraInfo(camerainfo);


            visualization_msgs::Marker cam_poly;	// create marker
            cam_poly.header = camerainfo.header;
            cam_poly.action = visualization_msgs::Marker::ADD;
            cam_poly.id = 0;
            cam_poly.ns = "fov";
            cam_poly.type = visualization_msgs::Marker::LINE_STRIP;	// ..as line strip
            cam_poly.scale.x = 0.01;
            cam_poly.color.g = 1.0; // set colors
            cam_poly.color.b = 1.0;
            cam_poly.color.a = 0.5; // set transparency

            // angle calculation

            // for example

            /*this should be fixed because this is only for testing and can be improved to a real situation in the robot !!!!*/

            double base_to_circle = 0.167;
            double base_to_camera = 1;


            min_distance = 0.01; max_distance = sqrt(pow(base_to_circle,2)+pow(base_to_camera,2));

        // calc 3D Points out of boundaries of image. Result are four points  (rect at 1m distance)
            cv::Point3d P_topleft = pinmodel.projectPixelTo3dRay(cv::Point(0, 0));
            cv::Point3d P_downright = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width, camerainfo.height));
            cv::Point3d P_topright = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width, 0));
            cv::Point3d P_downleft = pinmodel.projectPixelTo3dRay(cv::Point(0, camerainfo.height));
            cv::Point3d P_center = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width/2, camerainfo.height/2));
            cv::Point3d P_circle = pinmodel.projectPixelTo3dRay(cv::Point(center_x, center_y));
            if (!debug_mode && circle_detection_finished) {
                cv::Point3d P_circle = pinmodel.projectPixelTo3dRay(cv::Point(center_x, average_y_points));
            }

            // project rect into desired distances (min_distance and max_distance)

            // vertices front rect
            geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8, p9 , p10, p_ref;
            p1.x = P_topleft.x * min_distance; p1.y = P_topleft.y * min_distance; p1.z = P_topleft.z * min_distance;

            p2.x = P_topright.x * min_distance; p2.y = P_topright.y * min_distance; p2.z = P_topright.z * min_distance;

            p3.x = P_downright.x * min_distance; p3.y = P_downright.y * min_distance; p3.z = P_downright.z * min_distance;

            p4.x = P_downleft.x * min_distance; p4.y = P_downleft.y * min_distance;p4.z = P_downleft.z * min_distance;

            // vertices rear rect
            p5.x = P_topleft.x * max_distance; p5.y = P_topleft.y * max_distance; p5.z = P_topleft.z * max_distance;

            p6.x = P_topright.x * max_distance; p6.y = P_topright.y * max_distance; p6.z = P_topright.z * max_distance;

            p7.x= P_downright.x * max_distance; p7.y= P_downright.y * max_distance;p7.z= P_downright.z * max_distance;

            p8.x= P_downleft.x * max_distance; p8.y= P_downleft.y * max_distance; p8.z= P_downleft.z * max_distance;

            p10.x = P_center.x * max_distance; p10.y = P_circle.y * max_distance; p10.z = P_circle.z * max_distance;

            //set the starting point of the camera color (please calibrate(?))
            p9.x = P_center.x * min_distance; p9.y = P_center.y * min_distance; p9.z = P_center.z * min_distance;

            p_ref.x = P_center.x * max_distance; p_ref.y = P_center.y * max_distance; p_ref.z = sqrt(pow(base_to_circle,2)+pow(base_to_camera,2));

            // to make 45 degrees angle, base_circle and base_camera = 1:1
            // base_to_camera = 0.5, or base_to_circle = 1.4, the only possible way is base_to_camera = 0.5 with projection.


            // get reference angle for calculation the degree of camera
            double angle_ref = atan(base_to_circle/base_to_camera)*180.0/PI;

            // get angle from detected circle line with the reference line
            double m_line = (p10.y - p9.y)/(max_distance - min_distance);  // getting the gradient from the detected circle
            double m_ref = (p_ref.y -p9.y)/(max_distance - min_distance);  // getting the gradient from the reference

            double angle;
            double x = std::abs((m_line-m_ref)/(1.0+(m_line*m_ref))); // using equation to search angle from known 2 lines.

            angle = atan(x)*180.0/PI; // in degrees

            // this is the decision where to add or where to substract, this also needs to be checked.

            if (p10.y > p_ref.y) {
                angle = angle + angle_ref;
            }
            else
            {
                angle = std::abs(angle - angle_ref);
            }

            // std::cout << "the degree is  " << angle << std::endl;

            //euler to quaternion
            angle = 90.0 - angle;  // this is only for one case, needs to be fixed for another case.


            // reference : http://wiki.alioth.net/index.php/Quaternion



            // send angle to getPose function to publish the orientation of the camera
            getPose(angle,camerainfo);

            // push back points to get line polynom
            cam_poly.points.push_back(p1); cam_poly.points.push_back(p2); cam_poly.points.push_back(p3);
            cam_poly.points.push_back(p4); cam_poly.points.push_back(p1); cam_poly.points.push_back(p5);
            cam_poly.points.push_back(p6); cam_poly.points.push_back(p7); cam_poly.points.push_back(p8);
            cam_poly.points.push_back(p5); cam_poly.points.push_back(p8); cam_poly.points.push_back(p4);
            cam_poly.points.push_back(p3); cam_poly.points.push_back(p7); cam_poly.points.push_back(p6);
            cam_poly.points.push_back(p2); cam_poly.points.push_back(p9); cam_poly.points.push_back(p10);
            cam_poly.points.push_back(p9); cam_poly.points.push_back(p_ref);

            pub_marker_.publish(cam_poly);



    }

    void Hough::getPose(double angle,sensor_msgs::CameraInfo camerainfo)
    {
        geometry_msgs::PoseStamped ps;
        std_msgs::Header header;
        geometry_msgs::Pose pose;

		double position_x = 0; double position_y = -0.0325; double position_z = 1; // starting position of the camera link

        double rotation_y = cos ( angle * PI / 180.0 ); // should be rotation_w, but because the orientation of the camera we need to change
        double rotation_w = sin ( angle * PI / 180.0 );

    	static tf::TransformBroadcaster br;
	    tf::Transform transform_base_camera;
        double time = camerainfo.header.stamp.toSec();

	    transform_base_camera.setOrigin(tf::Vector3(position_x,position_y,position_z));
        // transform_base_camera.setRotation(tf::Quaternion(rotation_x,rotation_y,rotation_z,rotation_w));
        angle = (angle+180) * PI / 180;  // this is only for one case, needs to be fixed for another case.

        tf::Quaternion w;
		w.setRPY(0,-angle,0);  // set the 2DOF orientation in Roll Pitch and Yaw. The orientation needed is only the pitch.
		transform_base_camera.setRotation(w);

        br.sendTransform(tf::StampedTransform(transform_base_camera,ros::Time(time),"base_link","camera_link_pose"));

        if (!debug_mode && circle_detection_finished) { // circle detection is finished after publishing a TF in non-debugging mode!
            do_circle_detection = true;
            circle_detection_finished = false;
        }

    }

}
