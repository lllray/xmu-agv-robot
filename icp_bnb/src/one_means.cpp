//
// Created by ray on 19-7-5.
//
#include <one_means.h>

pair<Eigen::Vector3d,int> scan_to_world(int i,double dist){
    //double x,y;
    double theta,rho;
    pair<Eigen::Vector3d,int> output;
    theta = (i/2.0 - 45)*PI/180;
    rho = dist;
    output.first[0] = rho*cos(theta);
    output.first[1] = rho*sin(theta);
    output.first[2] = 0;
    output.second = i;
    return output;

}


vector<Eigen::Vector3d> one_means(sensor_msgs::LaserScan scan){
    vector<Eigen::Vector3d> pts_cloud;
    pts_cloud.reserve (scan.ranges.size());
    pts_cloud.clear();

    vector<pair<Eigen::Vector3d,int>> search_map;
    pair<Eigen::Vector3d,int> first_point;
    first_point.first=Eigen::Vector3d(0,0,0);
    int center_n=(int)scan.ranges.size()/2;
    first_point.second=center_n;
    search_map.push_back(first_point);
    bool fisrt_find=false;
    while(search_map.size()>0) {
        pair<Eigen::Vector3d, int> last_p = search_map.back();//获取最后一个元素
        search_map.pop_back();//获取后删除
        if (fisrt_find) {
            //左边的种子点向左搜索
            if (last_p.second <= center_n) {
                for (double i = last_p.second - 1; i > 0; i--) {
                    //如果在一定距离范围内找到点
                    if ((scan.ranges[(int)i] > 0) && (scan.ranges[(int)i] < SEARCH_MAX_DISTACCE)) {
                        //将点转换到三维坐标
                        pair<Eigen::Vector3d, int> current_p = scan_to_world((int)i, scan.ranges[(int)i]);
                        //求该点与上一个点的距离
                        double dist = (last_p.first - current_p.first).norm();
                        //判断间距是否符合要求
                        if (dist < SEARCH_MIN_DISTANCE) {
                            //该点保存入点云，并加入种子点
                            search_map.push_back(current_p);
                            pts_cloud.push_back(current_p.first);
                            break;
                        }
                    }
                }

            }
            //右边的种子点向右搜索
            if (last_p.second >= center_n) {
                for (double i = last_p.second + 1; i < scan.ranges.size(); i++) {
                    //如果在一定距离范围内找到点
                    if ((scan.ranges[(int)i] > 0) && (scan.ranges[(int)i] < SEARCH_MAX_DISTACCE)) {
                        //将点转换到三维坐标
                        pair<Eigen::Vector3d, int> current_p = scan_to_world((int)i, scan.ranges[(int)i]);
                        //求该点与上一个点的距离
                        double dist = (last_p.first - current_p.first).norm();
                        //判断间距是否符合要求
                        if (dist < SEARCH_MIN_DISTANCE) {
                            //该点保存入点云，并加入种子点
                            search_map.push_back(current_p);
                            pts_cloud.push_back(current_p.first);
                            break;
                        }
                    }
                }
            }
        } else {
            for (int i = 0; i < SEARCH_ANGLE_OFFSET; i++) {
                if ((scan.ranges[center_n - i] > 0) && (scan.ranges[center_n - i] < SEARCH_MAX_DISTACCE)) {
                    pair<Eigen::Vector3d, int> current_p = scan_to_world(center_n - i, scan.ranges[center_n - i]);
                    search_map.push_back(current_p);
                    pts_cloud.push_back(current_p.first);
                    fisrt_find = true;
                    center_n = current_p.second;
                    break;
                } else if ((scan.ranges[center_n + i] > 0) && (scan.ranges[center_n + i] < SEARCH_MAX_DISTACCE)) {
                    pair<Eigen::Vector3d, int> current_p = scan_to_world(center_n + i, scan.ranges[center_n + i]);
                    search_map.push_back(current_p);
                    pts_cloud.push_back(current_p.first);
                    fisrt_find = true;
                    center_n = current_p.second;
                    break;
                }
            }
        }
    }

    return pts_cloud;

}
