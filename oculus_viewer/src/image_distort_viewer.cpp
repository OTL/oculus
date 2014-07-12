#include <ros/ros.h>
#include <oculus_msgs/HMDInfo.h>
#include <oculus_viewer/distort.h>
#include <oculus_viewer/viewer.h>

namespace oculus_viewer
{

    class ImageDistortViewer
    {
         public:
            ImageDistortViewer();
            void init();
            void HMDInfoCallback(const oculus_msgs::HMDInfoPtr& info);
            void show();
         private:
            DistortImage left_;
            DistortImage right_;
            ros::Subscriber sub_;
            bool use_display_;
            Viewer viewer_;
    };

    ImageDistortViewer::ImageDistortViewer(): use_display_(true), viewer_("oculus camera view")
    {

    }

    void ImageDistortViewer::init()
    {
        ros::NodeHandle node;
        std::string left_img, right_img;
        // left_.init("camera/left/image_raw");
        // right_.init("camera/right/image_raw");
        // left_.init("camera/left/image_rect");
        // right_.init("camera/right/image_rect");


        sub_ = node.subscribe("/oculus/hmd_info",
                            1,
                            &ImageDistortViewer::HMDInfoCallback,
                            this);
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("left_image", left_img, "camera/left/image_raw");
        private_node.param<std::string>("right_image", right_img, "camera/right/image_raw");
        
        ROS_INFO("Left image topic: %s", left_img.c_str());
        ROS_INFO("Right image topic: %s", right_img.c_str());

        left_.init(left_img);
        right_.init(right_img);


        
        int32_t offset_x = 0;
        private_node.param<int32_t>("display_offset_x", offset_x, 0);
        int32_t offset_y = 0;
        private_node.param<int32_t>("display_offset_y", offset_y, 0);
        viewer_.setDisplayOffset(offset_x, offset_y);

        private_node.param<bool>("use_display", use_display_, true);
    }

    void ImageDistortViewer::show()
    {
        if (use_display_)
        {
            if ((!right_.getImage().empty()) && (!left_.getImage().empty()))
            {
                viewer_.show(right_.getImage(), left_.getImage());
            }
        }
    }

    void ImageDistortViewer::HMDInfoCallback(const oculus_msgs::HMDInfoPtr& info)
    {
        if (info->horizontal_screen_size > 0)
        {
            double lens_center = 1 - 2 * info->lens_separation_distance / info->horizontal_screen_size;
            double scale = 1 + lens_center;
            left_.setK(info->distortion_K);
            left_.setScale(scale);
            left_.setOffset(-lens_center);
            right_.setK(info->distortion_K);
            right_.setScale(scale);
            right_.setOffset(lens_center);
        }
    }

}  // namespace oculus_viewer


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_distort_viewer");
    try
    {
        oculus_viewer::ImageDistortViewer dis;
        dis.init();
        while(ros::ok())
        {
            cv::waitKey(100);
            ros::spinOnce();
            dis.show();
        }
    }
    catch(ros::Exception& e)
    {
        ROS_ERROR("ros error: %s", e.what());
    } catch(...)
    {
        ROS_FATAL("unexpected error");
    }
}
