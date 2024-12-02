#include <opencv2/core/core.hpp>
#include <vector>
#include <mutex>
#include <memory>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "dynorb_interface/srv/feed_frame.hpp"

#include "DynamObj.h"

namespace ORB_SLAM2
{

class PointFilter;

typedef size_t dynObjID;

//used to filter dynamic objects and accept only static objects
class NegatePointFilter : public PointFilter
{
    public:
        NegatePointFilter(PointFilter* p) : filter(p) {};
        //returns true if point in image is rejected by the filter, false o.w.
        bool operator()(cv::Point2f& point) override {return !(*filter)(point);};
    private:
        PointFilter* filter;
};

/*
We maintain an interface with the SAM2 model that continuously feeds it a stream of
frames from the camera and requests categorization of object in [0, num_dyn_objs)
This class interfaces with the SAM2 ROS node and manages the feeding of frames to ros
*/
class DynamObjTracker : public PointFilter, public rclcpp::Node
{
    public:
        /*
        initializes the SAM2 model with the initial frame:
            - calculates the masks for the initial frame
            - creates a DynamObjectTracker for every dynamic object
        */
        DynamObjTracker() : 
        is_static(std::make_shared<NegatePointFilter>(this)),
        Node("dynam_obj_tracker")
        {
            this->client = this->create_client<dynorb_interface::srv::FeedFrame>("feed_frame");

            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully initialized.");
        };

        /*
        Sends a frame to SAM2 to process. 
        If check=True, checks if any new dynamic objects are detected, and adds them if necessary
        */
        void process_frame(const cv::Mat frame){
            //call the sam2 ros service to process frame
            auto request = std::make_shared<dynorb_interface::srv::FeedFrame::Request>();

            RCLCPP_INFO(this->get_logger(), "Recieved request");
            
            std_msgs::msg::Header hdr;
            cv_bridge::CvImage(hdr, "rgb8", frame).toImageMsg(request->frame);

            auto result = client->async_send_request(request);

            // Wait for the result (blocking call)
            auto response = result.get();

            if (response) {
                // Process the response
                RCLCPP_INFO(this->get_logger(), "Frame received");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call sam2 service");
            }

            // std::shared_ptr<void const> tracked_object;
            frame_mutex.lock(); 
            this->current_seg_frame = cv_bridge::toCvCopy(response->seg, "mono8")->image;
            frame_mutex.unlock(); 

            RCLCPP_INFO(this->get_logger(), "Successfully extracted frame.");
        };

        /*
        Takes in a pixel image in the current frame
        Returns -1 if not part of dynamic object. 
        Otherwise returns the DynamObj id related to the object 
        (and creates one if does not exist in the system)
        */
        dynObjID get_object_at(cv::Point2f& point)
            {frame_mutex.lock(); 
            int cls = current_seg_frame.at<int>((int)point.x, (int)point.y);
            frame_mutex.unlock(); 
            return cls;};

        //number of dynamic objects in the system
        dynObjID get_num_obj(){return objects.size();};

        //returns true if point in image is on a dynamic object, false o.w.
        //rn is just a dummy function that removes all the keypoints in top left corner
        //point.x<200 && point.y<200
        bool operator()(cv::Point2f& point) override {return this->get_object_at(point)>=0;};

        //filter used to negate the dynamic point filter (used to accept points on static objects)
        std::shared_ptr<NegatePointFilter> is_static;

    private:
        rclcpp::Client<dynorb_interface::srv::FeedFrame>::SharedPtr client;

        //current frame post processing
        cv::Mat current_seg_frame;

        //ensures we don't process a frame while interacting with the mask
        std::mutex frame_mutex;

        //list of dynamic objects in the system
        std::map<dynObjID, DynamObj*> objects; //TODO -> maybe exists better rep

        //idx of the current net dynamic object
        dynObjID dynam_obj_idx;
};
}