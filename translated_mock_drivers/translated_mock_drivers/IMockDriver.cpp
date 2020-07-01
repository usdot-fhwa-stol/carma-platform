#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
using namespace std;


namespace mock_drivers {

class IMockDriver{
    
    public:
        /**
        * Function which should be called in the onStart function of a containing ROS Node
        * @param node The node which is being started
         */
        
        virtual void onStart(ros::NodeHandle node);


         /**
          * Function which should be called in the handleInterruptedException of a CancellableLoop
          */
        virtual void onInterruption();

        /**
         * Returns the name of this driver
         * @return the node name
         */
        virtual string getName();

         /**
          * Reads simulated data from a file and publishes it.
          * A driver data file is a csv file which can fill out all the elements of the ros messages published by a driver
          * All data files must have the first column be designated for sample id number.
          * Data rows with the same sample id number will be published during the same call to readAndPublishData
          * Other data in a data file is driver dependant and defined by the column headers and message files
          */
        virtual void readAndPublishData();

         /**
          * Publishes the status of this driver
          */
        virtual void publishDriverStatus();

        /**
         * Gets a list of topics names representing the api of this driver
         */
        virtual list<string> getDriverAPI();

        /**
         * Gets the delay in ms between when data should be published from this driver
         */
        virtual long getPublishDelay();


    
};
}
