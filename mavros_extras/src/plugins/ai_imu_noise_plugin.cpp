#include <mavros/mavros_plugin.h>
#include <ai_msgs/ImuNoise.h>

#include <cstring>

namespace mavros {
namespace std_plugins {

class AiImuNoisePlugin : public plugin::PluginBase
{
public:
	AiImuNoisePlugin() : PluginBase() {}

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// GLOBAL node handle (IMPORTANT)
		ros::NodeHandle nh;

		// Subscribe to your ROS topic
		noise_sub_ = nh.subscribe(
			"/mavros/ai/imu_noise",
			10,
			&AiImuNoisePlugin::noise_cb,
			this
		);

		ROS_INFO("[AiImuNoisePlugin] Initialized and subscribed to /mavros/ai/imu_noise");
	}

	Subscriptions get_subscriptions() override {
		return {};
	}

private:
	ros::Subscriber noise_sub_;

	void noise_cb(const ai_msgs::ImuNoise::ConstPtr& msg)
	{
		ROS_INFO_THROTTLE(1.0, "[AiImuNoisePlugin] Received AI IMU noise");

		auto fcu_link = UAS_FCU(m_uas);

		if (!fcu_link) {
			ROS_WARN("[AiImuNoisePlugin] FCU link not ready");
			return;
		}

		// MAVLink message parameters (must match your XML exactly)
		static constexpr uint32_t MSG_ID = 50001;
		static constexpr uint8_t MSG_LEN = 24;   // 6 floats
		static constexpr uint8_t MSG_CRC = 11;   // MUST match XML

		mavlink::mavlink_message_t mav_msg{};

		float payload[6] = {
			static_cast<float>(msg->accel_noise[0]),
			static_cast<float>(msg->accel_noise[1]),
			static_cast<float>(msg->accel_noise[2]),
			static_cast<float>(msg->gyro_noise[0]),
			static_cast<float>(msg->gyro_noise[1]),
			static_cast<float>(msg->gyro_noise[2])
		};

		// Copy payload
		memcpy(_MAV_PAYLOAD_NON_CONST(&mav_msg), payload, MSG_LEN);

		mav_msg.msgid = MSG_ID;

		// Finalize MAVLink message
		mavlink::mavlink_finalize_message(
			&mav_msg,
			fcu_link->get_system_id(),
			fcu_link->get_component_id(),
			MSG_LEN,
			MSG_LEN,
			MSG_CRC
		);

		// Send to PX4
		fcu_link->send_message_ignore_drop(&mav_msg);

		ROS_INFO_THROTTLE(1.0,
			"[AiImuNoisePlugin] Sent MAVLink msg accel[%.3e %.3e %.3e]",
			payload[0], payload[1], payload[2]
		);
	}
};

}  // namespace std_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
	mavros::std_plugins::AiImuNoisePlugin,

	mavros::plugin::PluginBase
)