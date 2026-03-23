#include <mavros/mavros_plugin.h>
#include <ai_msgs/ImuNoise.h>

#include <cstring>

namespace mavros {
namespace std_plugins {

class AiImuNoisePlugin : public plugin::PluginBase
{
public:
	AiImuNoisePlugin() : PluginBase(), nh("~") {}

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		noise_imu_sub = nh.subscribe(
			"/ai/imu_noise",
			10,
			&AiImuNoisePlugin::noise_cb,
			this);
	}

	Subscriptions get_subscriptions() override {
		return {};
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber noise_imu_sub;

	void noise_cb(const ai_msgs::ImuNoise::ConstPtr& msg)
	{
		static constexpr uint32_t AI_IMU_NOISE_MSG_ID = 50001;
		static constexpr uint8_t AI_IMU_NOISE_LEN = 24;
		static constexpr uint8_t AI_IMU_NOISE_CRC = 11;

		mavlink::mavlink_message_t mav_msg {};
		auto fcu_link = UAS_FCU(m_uas);
		const float payload[] = {
			static_cast<float>(msg->accel_noise[0]),
			static_cast<float>(msg->accel_noise[1]),
			static_cast<float>(msg->accel_noise[2]),
			static_cast<float>(msg->gyro_noise[0]),
			static_cast<float>(msg->gyro_noise[1]),
			static_cast<float>(msg->gyro_noise[2])
		};

		memcpy(_MAV_PAYLOAD_NON_CONST(&mav_msg), payload, AI_IMU_NOISE_LEN);
		mav_msg.msgid = AI_IMU_NOISE_MSG_ID;
		mavlink::mavlink_finalize_message(
			&mav_msg,
			fcu_link->get_system_id(),
			fcu_link->get_component_id(),
			AI_IMU_NOISE_LEN,
			AI_IMU_NOISE_LEN,
			AI_IMU_NOISE_CRC);

		fcu_link->send_message_ignore_drop(&mav_msg);
	}
};

}  // namespace std_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AiImuNoisePlugin, mavros::plugin::PluginBase)