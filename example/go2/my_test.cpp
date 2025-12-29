#include <cmath>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <map>
#include <functional>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

float dt = 0.005;

class Custom
{
public:
    enum ActionPhase
    {
        PHASE_PREPARE = 0, 
        //动作枚举

    };

    Custom()
    {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

       
        action_durations_ = {
            {PHASE_PREPARE, 3.0}, 
            //动作需要完成时间 只包括该动作时间，不包括恢复时间
        };
        action_names_ = {
            {PHASE_PREPARE, "准备状态(静止站立)"},
            //动作名称
        };

        current_phase_ = PHASE_PREPARE;
        input_enabled_ = false;
        flag = 0;
        PrintActionMenu();
    };

    void RobotControl()
    {
        ct += dt;
        switch (current_phase_)
        {
        case PHASE_PREPARE:
            PrepareState();
        //填写动作执行方法

        default:
            sport_client.StopMove();
        }

        CheckPhaseTimeout();
    };

    void CheckPhaseTimeout()
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if(!movement_finish_){
            auto time = action_durations_[current_phase_];
            if(ct - phase_start_time_>=time){
                movement_finish_ = true;
                current_phase_ = PHASE_PREPARE;
                phase_start_time_ = ct;
                flag = 0;
            }
        }else if(movement_finish_&&!(input_enabled_)){
            auto time = action_durations_[PHASE_PREPARE];
            if(ct - phase_start_time_>=time){
                input_enabled_ = true;
            }
        }
    }
    bool SetAction(int action_id)
    {
        std::lock_guard<std::mutex> lock(input_mutex_);

        if (action_id == 0)
        {
            std::cout << "紧急停止！回归静止站立..." << std::endl;
            current_phase_ = PHASE_PREPARE;
            phase_start_time_ = ct;
            movement_finish_ = true;
            input_enabled_ = false; 
            flag = 0;
            return true;
        }

        if (action_id >= 1 && action_id <= 16)
        {
            ActionPhase new_phase = static_cast<ActionPhase>(action_id);
            if (action_names_.find(new_phase) != action_names_.end())
            {
                std::cout << "准备执行动作: " << action_names_[new_phase]
                          << " (持续时间: " << action_durations_[new_phase] << "秒)" << std::endl;
                std::cout << "先进入准备状态..." << std::endl;

                current_phase_ = new_phase;
                phase_start_time_ = ct;
                movement_finish_ = false;
                input_enabled_ = false;
                flag = 0;
                return true;
            }
        }
        return false;
    }


    void PrintActionMenu()
    {
        std::cout << "\n=== Unitree Go2 动作控制菜单 ===" << std::endl;
        for (int i = 1; i <= 17; ++i)
        {
            ActionPhase phase = static_cast<ActionPhase>(i);
            std::cout << i << ". " << action_names_[phase]
                      << " (" << action_durations_[phase] << "秒)" << std::endl;
        }
        std::cout << "0. 紧急停止" << std::endl;
        std::cout << "请输入动作编号: ";
    }


    bool IsInputEnabled() const
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        return input_enabled_;
    }


    std::string GetCurrentStatus() const
    {
        if (current_phase_ == PHASE_PREPARE)
            return "准备状态";
        if (action_names_.find(current_phase_) != action_names_.end())
        {
            return "执行中: " + action_names_.at(current_phase_);
        }
        return "未知状态";
    }

private:

    void PrepareState()
    {
        if(flag == 0){
            sport_client.StopMove();
            flag = 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        sport_client.BalanceStand();
    }

    void HighStateHandler(const void *message)
    {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    };

 
    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

    ActionPhase current_phase_ = PHASE_PREPARE;

    double phase_start_time_ = 0;
    double ct = 0;

    std::atomic<bool> input_enabled_{false};
    std::atomic<bool> movement_finish_{false};
    mutable std::mutex input_mutex_;

    std::map<ActionPhase, double> action_durations_;
    std::map<ActionPhase, std::string> action_names_;

    bool flag;
};


void InputThread(Custom *custom)
{
    while (true)
    {
        int action_id;
        std::cout << "请输入动作编号 (1-16, 0=紧急停止): ";
        std::cin >> action_id;

        if (!(action_id == 0 || custom->IsInputEnabled()))
        {
            std::cout << "当前状态: " << custom->GetCurrentStatus()
                      << " | 输入0紧急停止: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        else if (!(action_id >= 0 && action_id <= 16))
        {
            std::cout << "无效的动作编号! 请输入1-17之间的数字" << std::endl;
            custom->PrintActionMenu();
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        custom->SetAction(action_id);

        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    Custom custom;

    sleep(1);

    unitree::common::ThreadPtr controlThread =
        unitree::common::CreateRecurrentThread(dt * 1000000,
                                               std::bind(&Custom::RobotControl, &custom));

    std::thread inputThread(InputThread, &custom);

    inputThread.join();

    return 0;
}