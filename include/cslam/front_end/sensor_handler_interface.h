#ifndef _ISENSORHANDLER_H_
#define _ISENSORHANDLER_H_

namespace cslam
{
    /**
     * @brief Interface class for handling sensor data
     * 
     */
    class ISensorHandler
    {
    public:
        /**
         * @brief Virtual destructor
         */
        virtual ~ISensorHandler() {};

        /**
         * @brief Process new data callback
         * 
         */
        virtual void process_new_sensor_data() = 0;
    };
} // namespace cslam
#endif