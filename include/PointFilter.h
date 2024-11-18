// Abstract filter function used by ORBExtractor to determine what pts to keep
namespace ORB_SLAM2
{
    class PointFilter{
        public:
            virtual bool operator()(cv::Point2f& pt){return true;};
    };
}