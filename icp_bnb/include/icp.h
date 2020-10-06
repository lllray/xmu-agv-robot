// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef ICP_H
#define ICP_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include "bits/stdc++.h"
namespace LIDAR
{

	struct Queue{
		int n;
		double squared_dist_Estimate;
	};
	struct cmp{
		bool operator() (const Queue s1, const Queue s2) {
			//小的在前
			return s1.squared_dist_Estimate > s2.squared_dist_Estimate;
		}
		};
	struct State{
	    double x;
	    double y;
	    double a;
		int bnb_n;
		double sum_squared_dist;
//		int np_match;
		double score;
//		double theory_score;
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
	};
class ICP{
public:
	ICP();
	
	static bool findTransformation(const std::vector<Eigen::Vector3d>& pts_model,
		const std::vector<Eigen::Vector3d>& pts_cloud,
		double n_iters, double epsilon, double min_err,
		Eigen::Matrix3d& R, Eigen::Vector3d& t,Eigen::Vector3d& tt
	);
	
}; //class ICP
	
	
} // namespace LIDAR

#endif
