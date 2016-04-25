#include <ros/ros.h>
#include <std_msgs/String.h>
#include <deque>

namespace {

    // NOTE: Meant for basic types with a single 'data' element.
    template <class T>
    class Stabilizer
    {
    private:
        int count_;

        typedef std::deque<T> QueueType;
        QueueType queue_;

        ros::Subscriber sub_;
        ros::Publisher  pub_;

        ros::Time     last_received_;
        ros::Duration timeout_;
        ros::Timer    timer_;

    public:
        Stabilizer(ros::NodeHandle& n, ros::NodeHandle& np)
        {
            np.param("count", count_, 5);

            sub_ = n.subscribe("in", 10, &Stabilizer::msgCB, this);
            pub_ = n.advertise<T>("out", 10);

            double to;
            np.param("timeout", to, 1.0);
            timeout_ = ros::Duration(to);
            timer_ = n.createTimer(ros::Duration(to / 10),
                                   &Stabilizer::timerCB,
                                   this);

        }

        void msgCB(const T& msg)
        {
            queue_.push_back(msg);
            last_received_ = ros::Time::now();
            
            typedef std::map<typename T::_data_type, int> MapType;
            MapType map;

            while (queue_.size() > count_) {
                queue_.pop_front();
            }

            typedef typename QueueType::const_iterator It;
            for (It i = queue_.begin(); i != queue_.end(); ++i) {
                const T& msg = *i;
                map[msg.data]++;
            }

            T max_msg;
            int max_c = 0;
            typedef typename MapType::const_iterator MIt;
            for (MIt i = map.begin(); i != map.end(); ++i)
            {
                int c = i->second;
                if (c > max_c) {
                    max_c = c;
                    max_msg.data = i->first;
                }
            }
            pub_.publish(max_msg);
        }

        void timerCB(const ros::TimerEvent&)
        {
            if (queue_.empty()) {
                return;
            }

            if ((ros::Time::now() - last_received_) > timeout_) {
                queue_.clear();
            }
        }

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "input_stabilizer");

    ros::NodeHandle n, np("~");

    Stabilizer<std_msgs::String> node(n, np);

    ros::spin();

    return 0;
}

