/**
 * @brief test_add_message plugin
 * @file test_add_message.cpp
 * @author bdai <bdairobot@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/StoreTestMessageRos.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Test add message plugin
 *
 * Used to send test message to FCU and also get message from FCU
 * to FCU position and attitude estimators.
 *
 */
class TestAddMessagePlugin : public plugin::PluginBase {
public:
	TestAddMessagePlugin() : PluginBase(),
		test_nh("~test_add_message")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		// Advertise new message recived from FCU likc PX4
		test_add_message_pub = test_nh.advertise<mavros_msgs::StoreTestMessageRos>("test_message_from_FCU", 10);
		// Subscribe topic message going to send to FCU 
		test_add_message_sub = test_nh.subscribe("send_test_message", 10, &TestAddMessagePlugin::test_add_message_cb, this);
	}

	Subscriptions get_subscriptions()
	{	
		// comment if Rx disabled
		// Register callback function to copy mavlink message to ROS topic
		return { make_handler(&TestAddMessagePlugin::handle_test_add_message) };
	}

private:
	ros::NodeHandle test_nh;

	ros::Subscriber test_add_message_sub;
	ros::Publisher test_add_message_pub;

	// Registe callback function, subscribe ros topic and send to FCU
	void test_add_message_cb(const mavros_msgs::StoreTestMessageRos::ConstPtr &req)
	{
		mavlink::test_add_custom_message::msg::TEST_ADD_MESSAGE msg;
		
		msg.counter = req->counter;
		ROS_INFO("Send counter to FCU: %lu", msg.counter);
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	}

	// TEST_ADD_MESSAGE is an mavlink message, which is defined in Mavlink package
	void handle_test_add_message(const mavlink::mavlink_message_t *msg, mavlink::test_add_custom_message::msg::TEST_ADD_MESSAGE &rst)
	{
		auto test_add_msg = boost::make_shared<mavros_msgs::StoreTestMessageRos>();

		test_add_msg->header.stamp = ros::Time::now();
		test_add_msg->counter = rst.counter;
		ROS_INFO("Receive counter from FCU: %lu", test_add_msg->counter);
		test_add_message_pub.publish(test_add_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TestAddMessagePlugin, mavros::plugin::PluginBase)