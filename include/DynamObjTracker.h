#include <opencv2/core/core.hpp>
#include <vector>
#include <mutex>
#include <map>

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
This class interfaces with the Python header 
*/
class DynamObjTracker : public PointFilter
{
    public:
        /*
        initializes the SAM2 model with the initial frame:
            - calculates the masks for the initial frame
            - creates a DynamObjectTracker for every dynamic object
        */
        DynamObjTracker() : is_static(new NegatePointFilter(this)){};

        /*
        Sends a frame to SAM2 to process. 
        If check=True, checks if any new dynamic objects are detected, and adds them if necessary
        */
        void process_frame(const cv::Mat frame){
            //check if initial frame, do some sets/processing
            current_masked_frame = frame;
        };

        /*
        Takes in a pixel image in the current frame
        Returns -1 if not part of dynamic object. 
        Otherwise returns the DynamObj id related to the object 
        (and creates one if does not exist in the system)
        */
        dynObjID get_object_at(cv::Point2f point)
            {frame_mutex.lock(); frame_mutex.unlock(); return -1;};

        //number of dynamic objects in the system
        dynObjID get_num_obj(){return objects.size();};

        //returns true if point in image is on a dynamic object, false o.w.
        //rn is just a dummy function that removes all the keypoints in top left corner
        bool operator()(cv::Point2f& point) override {return point.x<300 && point.y<300;};

        //filter used to negate the dynamic point filter (used to accept points on static objects)
        NegatePointFilter* is_static;

    private:
        //current frame post processing
        cv::Mat current_masked_frame;
        bool initialized;
        //ensures we don't process a frame while interacting with the mask
        std::mutex frame_mutex;

        //list of dynamic objects in the system
        std::map<dynObjID, DynamObj*> objects; //TODO -> maybe exists better rep

        //idx of the current net dynamic object
        dynObjID dynam_obj_idx;
};
}