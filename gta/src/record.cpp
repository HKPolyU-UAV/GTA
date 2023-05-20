#include "include/record.h"

void gta_ns::RecordNodelet::err_cal(const geometry_msgs::PoseStamped::ConstPtr &obj_pose_gt, 
                                    const geometry_msgs::PoseStamped::ConstPtr &obj_pose_w)
{
    cout << "lala" << endl;

    err_x = obj_pose_gt->pose.position.x - obj_pose_w->pose.position.x;
    err_y = obj_pose_gt->pose.position.y - obj_pose_w->pose.position.y;
    err_z = obj_pose_gt->pose.position.z - obj_pose_w->pose.position.z;

    ofstream save("/home/luo/gta_final/src/GTA/gta/src/log/indoor/uav_gt.txt", ios::app);
    save << std::setprecision(5) << obj_pose_gt->pose.position.x << "," << obj_pose_gt->pose.position.y << "," << obj_pose_gt->pose.position.z << "," << endl;
    save.close();

    ofstream save_3("/home/luo/gta_final/src/GTA/gta/src/log/indoor/uav_tracked.txt", ios::app);
    save_3 << std::setprecision(5) << obj_pose_w->pose.position.x << "," << obj_pose_w->pose.position.y << "," << obj_pose_w->pose.position.z << "," << endl;
    save_3.close();

    ofstream save_2("/home/luo/gta_final/src/GTA/gta/src/log/indoor/uav_error.txt", ios::app);
    save_2 << std::setprecision(5) << err_x << "," << err_y << "," << err_z << "," << endl;
    save_2.close();
}