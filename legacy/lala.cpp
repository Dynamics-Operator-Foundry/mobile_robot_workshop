void gimbal_ctrl_driver::rc_callback(
    const mavros_msgs::msg::RCIn::SharedPtr msg
)
{
    
    rc_lala = *msg;

    if (rc_lala.channels[6] < 2000)
        desired_manual.setZero();
    else
    {
        using namespace std;
        // cout<<(rc_lala.channels[1] - 1515.0) / (2015.0-1015.0) * 2<<endl;
        // cout<<(rc_lala.channels[0] - 1515.0) / (2015.0-1015.0) * 2<<endl;
        double k = 4.0;
        desired_manual(0) = desired_manual(0) + k * (rc_lala.channels[1] - 1515.0) / (2015.0-1015.0) * 2;
        desired_manual(1) = desired_manual(1) + k * (rc_lala.channels[0] - 1515.0) / (2015.0-1015.0) * 2;



        // std::cout<<desired_manual<<std::endl<<std::endl;;

        if (desired_manual(0) > 20.0)
            desired_manual(0) = 20.0;
        else if (desired_manual(0) < -90.0)
            desired_manual(0) = -90.0;

        if (desired_manual(1) > 128.0)
            desired_manual(1) = 128.0;
        else if (desired_manual(1) < -128.0)
            desired_manual(1) = -128.0;
    }
    // for (auto what:rc_lala.channels)
    // {
    //     std::cout<<what<<std::endl;
    // }
    // std::cout<<std::endl;
}