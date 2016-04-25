#include <ros_episodic_memory/inputData.h>
#include <ros_episodic_memory/learningMode.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <vector>

namespace em_tools
{
    class SyncDelegate
    {
    public:
        virtual void update() = 0;
        virtual ~SyncDelegate() {}
    };

    class Synchronizer
    {
    private:
        SyncDelegate*   sd_;

        std::string     channel_name_;
        ros::Duration   timeout_;
        double          channel_relevance_; 
        double          activation_value_; 

        ros::Subscriber sub_msg_;

        std::string     last_msg_;
        ros::Time       last_recept_;
        bool            was_empty_;

    public:
        Synchronizer(ros::NodeHandle&   n, 
                     ros::NodeHandle&   np,
                     const std::string& name,
                     SyncDelegate*      sd):
            sd_(sd),
            channel_name_(name),
            was_empty_(false)
        {
            std::string topic_name;
            double      to;

            np.param("topic",
                     topic_name, 
                     std::string("null_topic"));
            np.param("timeout", 
                     to,
                     1.0);
            np.param("relevance",
                     channel_relevance_,
                     10.0);
            np.param("activation_value",
                     activation_value_,
                     1.0);

            timeout_ = ros::Duration(to);

            sub_msg_ = n.subscribe(topic_name,
                                   5,
                                   &Synchronizer::msgCB,
                                   this);
        }

        const std::string channelName()      const
        { return channel_name_; }
        const float       channelRelevance() const
        { return channel_relevance_; }
        const float       activationValue()  const
        { return activation_value_; }

        bool valid() const
        {
            if (last_msg_ == "") {
                return false;
            } else {
                ros::Duration elapsed = ros::Time::now() - last_recept_;
                return (elapsed < timeout_);
            }
        }

        bool getMsg(std::string& msg) const
        {
            if (valid()) {
                msg = last_msg_;
                return true;
            } else {
                return false;
            }
        }

        void update()
        {
            if (!valid()) {
                last_msg_ = "";
            }
        }

    private:
        void msgCB(const std_msgs::String msg)
        {
            last_recept_ = ros::Time::now();
            if (last_msg_ != msg.data) {
                last_msg_ = msg.data;
                if (sd_) {
                    sd_->update();
                }
            }
        }

    };

    class Node: public SyncDelegate
    {
    private:
        ros::Timer     timer_;
        ros::Publisher pub_input_;
        ros::Publisher pub_learn_;

        typedef boost::shared_ptr<Synchronizer> SyncPtr;
        typedef std::vector<SyncPtr>            SyncVec;
        SyncVec syncs_;

        bool pub_empty_;

        ros_episodic_memory::inputData last_msg_;

    public:
        Node(ros::NodeHandle& n, ros::NodeHandle& np)
        {
            double p;
            np.param("period", p, 1.0);

            np.param("pub_empty", pub_empty_, true);

            XmlRpc::XmlRpcValue channels;
            np.getParam("channels", channels);
            if (channels.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR("'channels' parameter is not a struct.");
                return;
            }

            typedef std::map<std::string, XmlRpc::XmlRpcValue>::iterator It;
            for (It i = channels.begin(); i != channels.end(); ++i) {
                const std::string    c_name = i->first;
                ROS_INFO("Parsing channel '%s'", c_name.c_str());
                ros::NodeHandle nc(np, "channels/" + c_name);
                syncs_.push_back(SyncPtr(new Synchronizer(n,
                                         nc,
                                         c_name,
                                         this)));
            }
            
            pub_input_ =
                n.advertise<ros_episodic_memory::inputData>("input_data",
                                                            5);
            pub_learn_ = n.advertise<ros_episodic_memory::learningMode>(
                    "force_change_learning_mode",
                    5);

            timer_ = n.createTimer(ros::Duration(p),
                                   &Node::timerCB,
                                   this);
        }

        ~Node()
        {
        }

        void update()
        {
            ros_episodic_memory::inputData msg;
            
            typedef SyncVec::iterator It; 
            for (It i = syncs_.begin(); i != syncs_.end(); ++i) {
                Synchronizer& s = **i;
                s.update();
                std::string data;
                if (s.getMsg(data)) {
                    msg.categoryName.push_back(data);
                    msg.channelName.push_back(s.channelName());
                    msg.channelRelevance.push_back(s.channelRelevance());
                    msg.activationValue.push_back(s.activationValue());
                }
            }

            if (msg.categoryName != last_msg_.categoryName) {
                if (msg.categoryName.empty()) {
                    ros_episodic_memory::learningMode lm;
                    lm.learningMode = 
                        ros_episodic_memory::learningMode::LEARNING;
                    pub_learn_.publish(lm);
                    ros::Duration(1.0).sleep();
                }
                
                if (!msg.categoryName.empty() || pub_empty_) {
                    pub_input_.publish(msg);
                }
                last_msg_ = msg;
            }
        }

    private:
        void timerCB(const ros::TimerEvent&)
        {
            update();
        }

    };
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "input_sync");

    ros::NodeHandle n, np("~");
    em_tools::Node node(n, np);

    ros::spin();

    return 0;
}

