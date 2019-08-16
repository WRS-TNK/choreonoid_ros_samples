/**
   A simple controller that inputs a joystic state by subscribing a ROS Joy topic
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

using namespace cnoid;

class JoyTopicSubscriberController : public SimpleController, public JoystickInterface
{
    SharedJoystick* sharedJoystick;
    ros::NodeHandle node;
    ros::Subscriber joySubscriber;
    std::mutex joyMutex;
    sensor_msgs::Joy tmpJoyState;
    sensor_msgs::Joy joyState;
    std::string topic_name;
    
public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
      
        sharedJoystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
        std::string tmp_name = io->optionString();
        for(int i=9; i<tmp_name.size(); i++){
          topic_name.push_back(tmp_name[i]);
        }

        // topic_name = io->optionString();
        
        sharedJoystick->setJoystick(this);
        return true;
    }

    virtual bool start() override
    {
      joySubscriber = node.subscribe(topic_name, 1, &JoyTopicSubscriberController::joyCallback, this);
        return (bool)joySubscriber;
    }

    virtual bool control() override
    {
        return false;
    }

    virtual void stop() override
    {
        joySubscriber.shutdown();
    }

    void joyCallback(const sensor_msgs::Joy& msg)
    {
        std::lock_guard<std::mutex> lock(joyMutex);
        tmpJoyState = msg;
    }

    virtual int numAxes() const override
    {
        return joyState.axes.size();
    }
    
    virtual int numButtons() const override
    {
        return joyState.buttons.size();
    }

    virtual bool readCurrentState() override
    {
        std::lock_guard<std::mutex> lock(joyMutex);
        joyState = tmpJoyState;
        return true;
    }

    virtual double getPosition(int axis) const override
    {
        if(axis < joyState.axes.size()){
            return joyState.axes[axis];
        }
        return 0.0;
    }

    virtual bool getButtonState(int button) const override
    {
        if(button < joyState.buttons.size()){
            return joyState.buttons[button];
        }
        return false;
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JoyTopicSubscriberController)
