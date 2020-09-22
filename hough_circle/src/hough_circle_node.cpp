
#include"hough_circle/hough_circle.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ipa_hough_detection");
     //Hough * node = new Hough(nh);
    ipa_hough_circle::Hough Hough;

    ros::Rate rate(5);
    ros::spin();

    return 0;

}