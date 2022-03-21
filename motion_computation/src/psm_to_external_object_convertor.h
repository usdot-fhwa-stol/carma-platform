template<class T>
class MessageToExternalObjectConvertor
{

    public:
        /**
         * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
         *
         */
        virtual ~MessageConvertor(){};

        virtual void convert(const T& in_msg, carma_perception_msgs::msg::ExternalObject& out_msg) = 0;

};