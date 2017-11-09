//
// MainPage.xaml.cpp
// Implementation of the MainPage class.
//

#include "pch.h"
#include "MainPage.xaml.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace GamepadNodeUwp;

using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::Foundation::Collections;
using namespace Windows::UI::Xaml;
using namespace Windows::UI::Xaml::Controls;
using namespace Windows::UI::Xaml::Controls::Primitives;
using namespace Windows::UI::Xaml::Data;
using namespace Windows::UI::Xaml::Input;
using namespace Windows::UI::Xaml::Media;
using namespace Windows::UI::Xaml::Navigation;

#define ROS_WARN
#define ROS_INFO

// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

MainPage::MainPage()
{
	InitializeComponent();

	Windows::System::Threading::ThreadPool::RunAsync(ref new Windows::System::Threading::WorkItemHandler([this](Windows::Foundation::IAsyncAction^ spAction) 
	{
		NodeCode();
	}));
}

class Joystick
{
protected:
	// bool open_;              

	Windows::Gaming::Input::IGamepad^ _connectedGamepad = nullptr;
	EventRegistrationToken GamepadAddedToken;
	EventRegistrationToken GamepadRemovedToken;

	double deadzone_;
	double autorepeat_rate_;  // in Hz.  0 for no repeat.
	double coalesce_interval_; // Defaults to 100 Hz rate limit.

public:
	Joystick()
	{}

	///\brief Opens joystick port, reads from port and publishes while node is active
	int main(int argc, char **argv, MainPage^ mainPage, Windows::UI::Xaml::Controls::TextBlock^ status)
	{
		UNREFERENCED_PARAMETER(argc);
		UNREFERENCED_PARAMETER(argv);


		HRESULT hr = S_OK;

		auto cbAdd = ref new Windows::Foundation::EventHandler<Windows::Gaming::Input::Gamepad ^>([this](Platform::Object^, Windows::Gaming::Input::IGamepad^ pad)
		{
			_connectedGamepad = pad;
			OutputDebugStringA("Gamepad connected!!!\r\n");
		});

		auto cbRemove = ref new Windows::Foundation::EventHandler<Windows::Gaming::Input::Gamepad ^>([this](Platform::Object^, Windows::Gaming::Input::IGamepad^ pad)
		{
			_connectedGamepad = nullptr;
			OutputDebugStringA("Gamepad removed!!!\r\n");
		});


		auto node = std::make_shared<rclcpp::Node>("joy_node");
		auto pub = node->create_publisher<sensor_msgs::msg::Joy>("joy");

		// Parameters
		node->get_parameter_or("deadzone", deadzone_, 0.05);
		node->get_parameter_or("autorepeat_rate", autorepeat_rate_, static_cast<double>(20));
		node->get_parameter_or("coalesce_interval", coalesce_interval_, 0.001);


		// Checks on parameters
		if (autorepeat_rate_ > 1 / coalesce_interval_)
			ROS_WARN("joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1 / coalesce_interval_);

		if (deadzone_ >= 1)
		{
			ROS_WARN("joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone by 32767, but this behavior is deprecated so you need to update your launch file.");
			deadzone_ /= 32767;
		}

		if (deadzone_ > 0.9)
		{
			ROS_WARN("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
			deadzone_ = 0.9;
		}

		if (deadzone_ < 0)
		{
			ROS_WARN("joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
			deadzone_ = 0;
		}

		if (autorepeat_rate_ < 0)
		{
			ROS_WARN("joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
			autorepeat_rate_ = 0;
		}

		if (coalesce_interval_ < 0)
		{
			ROS_WARN("joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
			coalesce_interval_ = 0;
		}

		// Parameter conversions
		double autorepeat_interval = 1 / autorepeat_rate_;
		ROS_INFO("autorepeat_interval: %f", autorepeat_interval);
		double scale = -1. / (1. - deadzone_) / 32767.;
		ROS_INFO("scale: %f", scale);
		double unscaled_deadzone = 32767. * deadzone_;
		ROS_INFO("unscaled_deadzone: %f", unscaled_deadzone);

		// event_count_ = 0;
		// pub_count_ = 0;
		// lastDiagTime_ = rclcpp::Time::now().toSec();

		Windows::Gaming::Input::Gamepad::GamepadAdded += cbAdd;
		Windows::Gaming::Input::Gamepad::GamepadRemoved += cbRemove;


		OutputDebugStringA("Entering big loop to publish joystick ... \r\n");

		// Big while loop opens, publishes
		while (rclcpp::ok())
		{
			// open_ = false;
			// diagnostic_.force_update();
			bool first_fault = true;
			auto timer_callback = []() -> void {};

			// Here because we want to reset it on device close.
			auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
			joy_msg->header.stamp.sec = 0;
			joy_msg->header.stamp.nanosec = 0;
			joy_msg->header.frame_id = "joy";
			joy_msg->axes.resize(6);
			joy_msg->axes[0] = joy_msg->axes[1] = joy_msg->axes[2] = joy_msg->axes[3] = joy_msg->axes[4] = joy_msg->axes[5] = 0;
			joy_msg->buttons.resize(6);
			joy_msg->buttons[0] = joy_msg->buttons[1] = joy_msg->buttons[2] = joy_msg->buttons[3] = joy_msg->buttons[4] = joy_msg->buttons[5] = 0;

			OutputDebugStringA("Entering loop to read joystick input ... \r\n");
			int i = 0;

			while (rclcpp::ok())
			{
				rclcpp::spin_some(node);

				auto connectedGamepad = _connectedGamepad;
				if (connectedGamepad != nullptr)
				{
					auto currentReading = connectedGamepad->GetCurrentReading();
					auto statusText = ref new Platform::String() +
						"Gamepad reading:\r\n\tLeftThumbstickX=" + currentReading.LeftThumbstickX.ToString() +
						"\r\n\tLeftThumbstickY=" + currentReading.LeftThumbstickY.ToString() +
						"\r\n\tRightThumbstickX=" + currentReading.RightThumbstickX.ToString() +
						"\r\n\tRightThumbstickY=" + currentReading.RightThumbstickY.ToString() +
						"\r\n\tButtons=" + currentReading.Buttons.ToString() +
						"\r\n";

					joy_msg->axes[2] = currentReading.LeftThumbstickX;
					joy_msg->axes[5] = currentReading.RightThumbstickY;

					auto buttonsSelection = currentReading.Buttons;

					bool isSelfDirectedOn = (buttonsSelection == Windows::Gaming::Input::GamepadButtons::A);
					bool isSelfDirectedOff = (buttonsSelection == Windows::Gaming::Input::GamepadButtons::B);
					bool isEnableButtonPressed =
						(buttonsSelection == Windows::Gaming::Input::GamepadButtons::LeftShoulder ||
						buttonsSelection == Windows::Gaming::Input::GamepadButtons::RightShoulder);

					joy_msg->buttons[0] = joy_msg->buttons[1] = joy_msg->buttons[2] = joy_msg->buttons[3] = joy_msg->buttons[4] = joy_msg->buttons[5] = 0;
					joy_msg->buttons[0] = isSelfDirectedOn;
					joy_msg->buttons[1] = isSelfDirectedOff;
					joy_msg->buttons[5] = isEnableButtonPressed;


					if (isEnableButtonPressed || isSelfDirectedOn || isSelfDirectedOff)
					{
						OutputDebugString(L"*********PUBLISH GAMEPAD READING*****************\r\n");
						pub->publish(joy_msg);

						statusText = ref new Platform::String(L"SENT: ") + statusText;
					}

					OutputDebugString(statusText->Data());
					mainPage->Dispatcher->RunAsync(Windows::UI::Core::CoreDispatcherPriority::Normal, ref new Windows::UI::Core::DispatchedHandler([=]() {
						try {
							status->Text = statusText;
						}
						catch (...)
						{
						}

					}));

				}
				else
				{
					auto statusText = ref new Platform::String(L"No Gamepad connected");
					OutputDebugString(statusText->Data());
					mainPage->Dispatcher->RunAsync(Windows::UI::Core::CoreDispatcherPriority::Normal, ref new Windows::UI::Core::DispatchedHandler([=]() {
						try {
							status->Text = statusText;
						}
						catch (...)
						{
						}

					}));
				}

				WaitForSingleObject(GetCurrentThread(), 20);
			}
		}

	Fail_Cleanup:
		Windows::Foundation::Uninitialize();
		return 0;
	}
};

void MainPage::NodeCode()
{
	char *argv[] = { 0 };
	int argc = 0;
	rclcpp::init(argc, argv);
	Joystick j;
	j.main(argc, argv, this, this->status);
}
