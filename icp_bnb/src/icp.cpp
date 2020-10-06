// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <icp_define.h>
#include <ros/ros.h>
#include <math.h>
using namespace std;
namespace LIDAR
{

ICP::ICP()
{

}

Eigen::Vector3d polar_to_2d(double r,double d){
    return Eigen::Vector3d(d*cos(r),d*sin(r),0.0);

}

//返回当前层新增加点个数
int get_point_nn(int n,int n_iter,int seed,int size){

    if(n_iter==n)return seed;
    return (int)(seed*(int)pow(2,(n_iter-n-1)));
}
//返回当前层步长
int get_step_nn(int point_n,int size){

    return (int)(size/point_n);
}
//返回遍历点的初始位置
int get_first_nn(int n,int n_iter,int seed,int size){
    return (size/seed)/(int)pow(2,n_iter-n)-1;
}
double sum_squared_dist_qf(double current_score,double last_score,int bnb_n)
{
    double e_sum_squared_dist=current_score-last_score;
    return current_score+=(e_sum_squared_dist*(bnb_n-1));
}

    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d theta)
    {
        // 计算旋转矩阵的X分量
        Eigen::Matrix3d R_x;
        R_x<<
        1.0, 0.0, 0.0,
        0.0, cos(theta[0]), -sin(theta[0]),
        0.0, sin(theta[0]), cos(theta[0]);


        // 计算旋转矩阵的Y分量
        Eigen::Matrix3d R_y;
        R_y<<
        cos(theta[1]), 0, sin(theta[1]),
        0, 1, 0,
        -sin(theta[1]), 0, cos(theta[1]);


        // 计算旋转矩阵的Z分量
        Eigen::Matrix3d R_z;
        R_z<<
        cos(theta[2]), -sin(theta[2]), 0,
        sin(theta[2]), cos(theta[2]), 0,
        0, 0, 1;

        // 合并 
        //Eigen::Matrix3d R = R_z * R_y * R_x;
        Eigen::Matrix3d R = R_z;
        return R;
    }

bool ICP::findTransformation ( const std::vector< Eigen::Vector3d >& pts_model, const std::vector< Eigen::Vector3d >& pts_cloud, double n_iters, double epsilon, double min_err, Eigen::Matrix3d& R, Eigen::Vector3d& t ,Eigen::Vector3d& tt)
{
	// default settings.
	const double min_err2 = min_err * min_err;
	const double factor = 9.0;//N倍平均距离阈值
	const int n_selected_pts = 64;
	//int model_k = pts_model.size() / n_selected_pts;	// step for select points.
	//if(model_k<1)model_k=1;
    int model_k =1;
	//if(step==0)step=1;
    static Eigen::Matrix3d R_temp;
    static Eigen::Vector3d t_temp;
	// two vectors for matched points.
    std::vector<Eigen::Vector3d> pts_cloud_matched;
	pts_cloud_matched.reserve ( pts_cloud.size() );//reserve the vector size
    std::vector<Eigen::Vector3d> pts_model_matched;
	pts_model_matched.reserve ( pts_model.size() );



	// used for search.
	std::vector <int> index (1);
	std::vector <float> squared_distance (1);
	
	// Dth
	double squared_distance_th = std::numeric_limits <double>::max ();
	double cur_squared_dist = 0.0;
	double last_squared_dist = std::numeric_limits<double>::max();

    // step 2. Get R and t.
    // step 2.1 find centor of model(X) and cloud(P)
    Eigen::Vector3d mu_x(0.0, 0.0, 0.0);
    Eigen::Vector3d mu_p(0.0, 0.0, 0.0);
    State best_state;
    double squared_distance_best_score=std::numeric_limits <double>::max ();
    for(size_t i = 0; i < pts_model.size(); i ++)
    {
        mu_x += pts_model.at(i);
    }
    for(size_t i = 0; i < pts_cloud.size(); i ++)
    {
        mu_p += pts_cloud.at(i);
    }
    mu_x = mu_x / double(pts_model.size());
    mu_p = mu_p / double(pts_cloud.size());
    // construct kd-tree for model cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud ( new pcl::PointCloud<pcl::PointXYZ> );

    for(size_t i = 0; i < pts_model.size(); i +=model_k)
    {
        pts_model_matched.push_back(pts_model.at(i)-mu_x);
        const Eigen::Vector3d& ptm = pts_model_matched.back();
        pcl::PointXYZ pt ( ptm[0], ptm[1], ptm[2] );
        model_cloud->push_back ( pt );
    }

    pcl::KdTreeFLANN<pcl::PointXYZ>* kd_tree = new pcl::KdTreeFLANN <pcl::PointXYZ>();
    kd_tree->setInputCloud ( model_cloud );

    for(size_t i = 0; i < pts_cloud.size(); i ++)
    {
        pts_cloud_matched.push_back(pts_cloud.at(i)-mu_p);
    }

    int seed_k=SEED_K;
    int bnb_iter=1;

    //在这里计算步长等参数
    for(int i=seed_k;i<pts_cloud_matched.size();i=seed_k*(int)pow(2,bnb_iter)){
        bnb_iter++;
    }
    int virtual_szie=seed_k*(int)pow(2,bnb_iter);
    int last_bnb_iter=0;
    if(bnb_iter>MAX_M)last_bnb_iter=bnb_iter-MAX_M;

    int nn_data[bnb_iter+1][3];
    for(int i=0;i<=bnb_iter;i++){
        int nn=bnb_iter-i;
        nn_data[nn][0]=get_point_nn(nn,bnb_iter,seed_k,virtual_szie);
        nn_data[nn][1]=get_step_nn(nn_data[nn][0],virtual_szie);//nn有问题
        nn_data[nn][2]=get_first_nn(nn,bnb_iter,seed_k,virtual_szie);
    }

    for ( int n = 0; n < n_iters; n ++ ) {
        priority_queue<Queue, vector<Queue>,cmp> pq;
        std::vector<State> state_map;
        //当前层
        int bnb_n=bnb_iter;
        int num=0;

        int search_angle_max=6;//*2
        int search_dist_max=2;//*2
        double search_angle_step=PI/8/pow(search_angle_max,n)/search_angle_max;//angle　略微扩大范围
        double search_dist_x_step=0.004/pow(search_dist_max,n)/search_dist_max; //一开始从搜索半径0.2m的范围，之后迭代，提高精度
        double search_dist_y_step=0.004/pow(search_dist_max,n)/search_dist_max; //一开始从搜索半径0.2m的范围，之后迭代，提高精度

        int bound_n=0;
        int finsh_n=0;
//        for(int x=-search_dist_max;x<=search_dist_max;x++)
//            for(int y=-search_dist_max;y<=search_dist_max;y++)
        for(int x=0;x<=0;x++)
            for(int y=0;y<=0;y++)
                for(int a=-search_angle_max;a<=search_angle_max;a++)
        {
                if(n!=0&&a==0&&x==0&&y==0)continue;//中心点在第二次开始不需要执行

                double x_value=x*search_dist_x_step;
                double y_value=y*search_dist_y_step;
                double a_value=a*search_angle_step;

                double sum_squared_dist = 0.0;
                double squared_dist_Estimate=0.0;
                Eigen::AngleAxisd rotation_vector (a_value,Eigen::Vector3d(0,0,1));
                Eigen::Matrix3d R_c=rotation_vector.toRotationMatrix()*R;
                Eigen::Vector3d t_c(x_value,y_value,0);
                t_c+=t;

                for ( size_t i = nn_data[bnb_n][2]; i < pts_cloud_matched.size(); i += nn_data[bnb_n][1] ) {

                    Eigen::Vector3d pt = R_c * pts_cloud_matched.at(i) + t_c;
                    pcl::PointXYZ pt_d(pt[0], pt[1], pt[2]);
                    //ros::Time begin_time = ros::Time::now ();
                    if (!kd_tree->nearestKSearch(pt_d, 1, index, squared_distance)) {
                        std::cerr << "ERROR: no points found.\n";
                        continue;
                    }
                    //判断是否有数据
                    if (squared_distance[0] < squared_distance_th) {
                        // add squared distance.
                        sum_squared_dist += squared_distance[0];//计算当前均值
                    }

                    //double clustering_time = (ros::Time::now () - begin_time).toSec ();
                    //ROS_INFO ("%f secs for clustering (%d clusters).", clustering_time, (int) pts_cloud_matched.size());
                } // for all pts_cloud
            //若推入数据成功
            {
                State current;
                double last_score=0;
                current.bnb_n=bnb_n;
                current.sum_squared_dist=sum_squared_dist;
                current.R=R_c;
                current.t=t_c;
                current.x= x;
                current.y= y;
                current.a= a;
                current.score=current.sum_squared_dist/pts_cloud_matched.size();
                if(current.bnb_n==last_bnb_iter){//如果到了最后一层

                    if( current.score<squared_distance_best_score) {
                        squared_distance_best_score = current.score;
                        best_state = current;
                        continue;
                    }
                }
                else{
                    if( current.score > squared_distance_best_score){
                        if(DEBUG_INFO) {
                            bound_n += current.bnb_n - last_bnb_iter;
                           // ROS_INFO("FIRST BOUND ONE bound_n=%d num=%d bnb_n=%d score=%f", bound_n, num, current.bnb_n,
                           //          current.score);
                        }
                        continue;
                    }
                }
                squared_dist_Estimate = sum_squared_dist_qf(current.score, last_score,current.bnb_n);//均值预测
                Queue queue={num,squared_dist_Estimate};
                pq.push(queue);
               //ROS_INFO("GET first current  num=%d bnb_n=%d score=%f",num,current.bnb_n,current.score);
                state_map.push_back(current);
                num++;
            }

        }


        while(pq.size()>0) {
            //从队列中返回第一个元素
            Queue current_queue = pq.top();
            //队列中删除第一个元素
            pq.pop();

            int c_n = current_queue.n;
            State current = state_map[c_n];

            if (current.score > squared_distance_best_score)//判断当前均值是否大于最小值
            {
                if(DEBUG_INFO) {
                    bound_n += current.bnb_n - last_bnb_iter;
                   // ROS_INFO("BOUND ONE bound_n=%d num=%d bnb_n=%d score=%f", bound_n, c_n, current.bnb_n,
                   //          current.score);
                }
                continue;
            }

            bnb_n = current.bnb_n - 1;
            double sum_squared_dist = 0.0;
            double squared_dist_Estimate=0.0;
            for (size_t i = nn_data[bnb_n][2]; i < pts_cloud_matched.size(); i += nn_data[bnb_n][1]) {

                Eigen::Vector3d pt = current.R * pts_cloud_matched.at(i) + current.t;

                pcl::PointXYZ pt_d(pt[0], pt[1], pt[2]);
                if (!kd_tree->nearestKSearch(pt_d, 1, index, squared_distance)) {
                    std::cerr << "ERROR: no points found.\n";
                    continue;
                }
                //判断是否有数据
                if (squared_distance[0] < squared_distance_th) {
                    // add squared distance.
                    sum_squared_dist += squared_distance[0];//计算当前均值

                }

            }
            // for all pts_cloud
            //若推入数据成功
            {
                double last_score=current.score;
                current.bnb_n = bnb_n;
                //current.np_match += np_match;
                current.sum_squared_dist += sum_squared_dist;
                current.score = current.sum_squared_dist / pts_cloud_matched.size();
                if (current.bnb_n == last_bnb_iter) {//如果到了最后一层
                    if (current.score < squared_distance_best_score) {
                        squared_distance_best_score = current.score;
                        best_state = current;
                        if(DEBUG_INFO) {
                            ROS_INFO("GET ONE BEST  num=%d bnb_n=%d a=%f x=%f y=%f score=%f", c_n, current.bnb_n,
                                     current.a, current.x, current.y, current.score);
                            //ROS_INFO("              a=%f x=%f y=%f score=%f",current.a,current.x,current.y);
                        }
                        continue;
                    }else{
                        if(DEBUG_INFO) {
                            finsh_n++;
                           // ROS_INFO("GET ONE FINNSH  num=%d finsh_n=%d bnb_n=%d score=%f", c_n, finsh_n, current.bnb_n,
                           //          current.score);
                        }
                        continue;
                    }

                }
                squared_dist_Estimate = sum_squared_dist_qf(current.score, last_score,current.bnb_n);//均值预测
                current_queue.squared_dist_Estimate = squared_dist_Estimate;
                pq.push(current_queue);
                state_map[c_n] = current;
            }
        }
        //若推入数据失败 暂时不考虑失败
        // step 3. Check if convergenced.
        cur_squared_dist = best_state.score;
        double squared_dist_change = last_squared_dist - cur_squared_dist;

        if((cur_squared_dist < min_err2)&&(USE_KD_TREE))
        {
            if(DEBUG_INFO) {
                ROS_INFO("cur_squared_dist=%f squared_dist_change=%f < epsilon=%f || cur_squared_dist:=%f < min_err2=%f",\
                          cur_squared_dist,   squared_dist_change,     epsilon,      cur_squared_dist,      min_err2);
            }
            break;
        }
        R=best_state.R;
        t=best_state.t;
		last_squared_dist = cur_squared_dist;
		//squared_distance_th = factor * cur_squared_dist;
        if(DEBUG_INFO) {

            ROS_INFO("n_iters=%f n=%d  cur_squared_dist:=%f",n_iters, n,cur_squared_dist);
        }
    } // for n_iters
    //double offset_k=-atan(mu_p[0]/mu_p[1]);
    Eigen::Vector3d rpy;
    Eigen::Vector3d car;
    rpy = R.eulerAngles(0, 1, 2);
    car[0]=CAR_LENGTH*sinf(rpy[2]);
    car[1]=CAR_LENGTH*cosf(rpy[2]);
    car[2]=0.0;
    //Eigen::AngleAxisd rotation_k (offset_k,Eigen::Vector3d(0,0,1));
    //Eigen::Matrix3d R_k=rotation_k.toRotationMatrix();
    tt=mu_p-t-t-car;
    t=mu_x-R*mu_p+t;

    if(DEBUG_INFO) {
        ROS_INFO("/*******************************/");

//        std::cout<< "R:"<<R<<std::endl;
//        std::cout<< "t:"<<t<<std::endl;
    }

    delete kd_tree;
    if(best_state.score>0.001)return false;
    return true;
} // findTransformation


} // namespace LIDAR
