/*
 * FILE: lidar_detector.cpp
 * -------------------------------------------------------------------------
 * Ported verbatim from the ROS1 LV-DOT onboard_detector/lidarDetector.cpp.
 * Only the include path was changed.
 */
#include "lvdot_core/lidar_detector.hpp"
namespace onboardDetector{
    lidarDetector::lidarDetector(){
        this->eps_ = 0.5;
        this->minPts_ = 10;
        this->cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void lidarDetector::setParams(double eps, int minPts){
        this->eps_ = eps;
        this->minPts_ = minPts;
    }

    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;
    }

    void lidarDetector::lidarDBSCAN(){
        if(!cloud_ || cloud_->empty()){
            this->clusters_.clear();
            this->bboxes_.clear();
            return;
        }

        if (this->minPts_ <= 0 || this->eps_ <= 0.0) {
            this->clusters_.clear();
            this->bboxes_.clear();
            return;
        }
        
        std::vector<Point> points;
        points.reserve(cloud_->size());
        for(size_t i=0; i<cloud_->size(); ++i){
            const auto & src = cloud_->points[i];
            if (!std::isfinite(src.x) || !std::isfinite(src.y) || !std::isfinite(src.z)) {
                continue;
            }
            Point p;
            p.x = src.x;
            p.y = src.y;
            p.z = src.z;
            p.clusterID = UNCLASSIFIED;
            points.push_back(p);
        }

        if (points.empty()) {
            this->clusters_.clear();
            this->bboxes_.clear();
            return;
        }

        DBSCAN dbscan(minPts_, eps_, points);
        dbscan.run();


        // find how many clusters
        int clusterNum = 0;
        for (size_t i=0; i<dbscan.m_points.size(); ++i){
            const onboardDetector::Point & pDB = dbscan.m_points[i];
            if (pDB.clusterID > clusterNum){
                clusterNum = pDB.clusterID;
            }
        }


        std::vector<onboardDetector::Cluster> clustersTemp;
        clustersTemp.resize(clusterNum);
        
        for(size_t i=0; i<dbscan.m_points.size(); ++i){
            if (dbscan.m_points[i].clusterID > 0){
                pcl::PointXYZ point;
                point.x = dbscan.m_points[i].x;
                point.y = dbscan.m_points[i].y;
                point.z = dbscan.m_points[i].z;
                clustersTemp[dbscan.m_points[i].clusterID-1].points->push_back(point);
                clustersTemp[dbscan.m_points[i].clusterID-1].cluster_id = dbscan.m_points[i].clusterID;
            }
        }
        this->clusters_ = clustersTemp;

        std::vector<onboardDetector::box3D> bboxesTemp;
        for(auto& cluster : this->clusters_){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster.points, centroid);
            cluster.centroid = centroid;
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cluster.points, minPt, maxPt);
            cluster.dimensions = Eigen::Vector3f(maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z); 

            onboardDetector::box3D bbox;
            bbox.x = centroid(0);
            bbox.y = centroid(1);
            bbox.z = centroid(2);
            bbox.x_width = maxPt.x - minPt.x;
            bbox.y_width = maxPt.y - minPt.y;
            bbox.z_width = maxPt.z - minPt.z;
            bboxesTemp.push_back(bbox);
        }
        this->bboxes_ = bboxesTemp;
    }

    std::vector<onboardDetector::Cluster>& lidarDetector::getClusters(){
        return this->clusters_;
    }

    std::vector<onboardDetector::box3D>& lidarDetector::getBBoxes(){
        return this->bboxes_;
    }
}
