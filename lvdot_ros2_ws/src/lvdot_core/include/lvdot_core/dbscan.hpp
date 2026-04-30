/*
 * FILE: dbscan.hpp
 * -------------------------------------------------------------------------
 * Ported verbatim from the ROS1 LV-DOT onboard_detector/dbscan.h.
 * Namespace, symbols and public API are unchanged.
 */
#ifndef LVDOT_CORE_DBSCAN_HPP
#define LVDOT_CORE_DBSCAN_HPP

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

namespace onboardDetector {

struct Point
{
    float x, y, z;
    int clusterID;
};

class DBSCAN
{
public:
    DBSCAN(unsigned int minPts, float eps, std::vector<Point> points)
    {
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN() {}

    int run();
    std::vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);

    int getTotalPointSize() { return m_pointSize; }
    int getMinimumClusterSize() { return m_minPoints; }
    int getEpsilonSize() { return m_epsilon; }

public:
    std::vector<Point> m_points;

private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

}  // namespace onboardDetector

#endif  // LVDOT_CORE_DBSCAN_HPP
