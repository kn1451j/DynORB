#include <opencv2/core/core.hpp>
#include <vector>
#include <mutex>

#include "System.h"
#include "DynamObj.h"

/*
We maintain an interface with the SAM2 model that continuously feeds it a stream of
frames from the camera and requests categorization of object in [0, num_dyn_objs)
This class interfaces with the Python header 
*/
class DynamObjTracker
{
    public:
        /*
        initializes the SAM2 model with the initial frame:
            - calculates the masks for the initial frame
            - creates a DynamObjectTracker for every dynamic object
        */
        SAM2::SAM2(const cv::Mat &initial_frame){};

        /*
        Sends a frame to SAM2 to process. 
        If check=True, checks if any new dynamic objects are detected, and adds them if necessary
        */
        void process_frame(const cv::Mat &initial_frame){};

        /*
        Takes in a pixel image in the current frame
        Returns nullptr if not part of dynamic object. 
        Otherwise returns reference to DynamObj
        */
        DynamObj& get_object_at(uint8_t x, uint8_t y)
            {frame_mutex.lock(); frame_mutex.unlock(); return nullptr;}

        //number of dynamic objects in the system
        std::uint_8 get_num_obj(){return objects.size();};

    private:
        //current frame post processing
        const cv::Mat current_masked_frame;
        //ensures we don't process a frame while interacting with the mask
        std::mutex frame_mutex;

        //list of dynamic objects in the system
        std::vector<DynamObj> objects; //TODO -> maybe exists better rep

        //idx of the current net dynamic object
        std::uint_8 dynam_obj_idx;
}