#ifndef __OCULUS_VIEWER_DISTORT_H__
#define __OCULUS_VIEWER_DISTORT_H__

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace oculus_viewer
{

    IplImage* barrel_dist(IplImage* img, double Cx, double Cy,double k0, double k1, double k2);

    class DistortImage
    {
        public:
            DistortImage();
            void init(const std::string& topic_name);
            const std::vector<float>& getK() const
            {
                return K_;
            }
            void setK(const std::vector<float>& K)
            {
                K_ = K;
            }
        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        const cv::Mat& getImage() const
        {
            return img_;
        }
        void setOffset(double offset)
        {
            offset_ = offset;
        }
        double getOffset() const
        {
            return offset_;
        }
        void setScale(double scale)
        {
            scale_ = scale;
        }
        double getScale() const
        {
            return scale_;
        }

        private:
            ros::NodeHandle nh_;
            image_transport::Subscriber sub_;
            image_transport::Publisher pub_;
            image_transport::ImageTransport it_;
            std::vector<float> K_;
            cv::Mat img_;
            double offset_;
            double scale_;
    };

}  // namespace oculus_viewer

#endif  // __OCULUS_VIEWER_DISTORT_H__
