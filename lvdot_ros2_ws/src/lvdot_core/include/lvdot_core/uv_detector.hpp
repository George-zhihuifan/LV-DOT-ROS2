/*
 * FILE: uv_detector.hpp
 * -------------------------------------------------------------------------
 * Ported from the ROS1 LV-DOT onboard_detector/uvDetector.h.
 * Only change: include paths updated to lvdot_core tree, and `using namespace
 * std` is scoped inside the `onboardDetector` namespace (the ROS1 header
 * relied on a global one inherited from kalmanFilter.h).
 */
#ifndef LVDOT_CORE_UV_DETECTOR_HPP
#define LVDOT_CORE_UV_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <math.h>
#include <vector>
#include <queue>

#include <Eigen/Dense>

#include "lvdot_core/box3d.hpp"
#include "lvdot_core/kalman_filter.hpp"

namespace onboardDetector {

using namespace std;  // preserved from the original header's transitive scope

class UVbox
{
public:
    int id;
    int toppest_parent_id;
    cv::Rect bb;

    UVbox();
    UVbox(int seg_id, int row, int left, int right);
};

class UVtracker
{
public:
    std::vector<cv::Rect> pre_bb;
    std::vector<cv::Rect> now_bb;
    std::vector<vector<cv::Point2f> > pre_history;
    std::vector<vector<cv::Point2f> > now_history;
    std::vector<kalman_filter> pre_filter;
    std::vector<kalman_filter> now_filter;
    std::vector<cv::Rect> now_bb_D;
    std::vector<box3D> now_box_3D;
    std::deque<deque<box3D>> now_box_3D_history;
    std::deque<deque<box3D>> pre_box_3D_history;
    float overlap_threshold;

    std::deque<std::deque<Eigen::MatrixXd>> pre_V;
    std::deque<std::deque<Eigen::MatrixXd>> now_V;

    std::deque<std::deque<int>> pre_count;
    std::deque<std::deque<int>> now_count;

    std::vector<box3D> fixed_box3D;

    UVtracker();

    void read_bb(vector<cv::Rect> now_bb, vector<cv::Rect> now_bb_D, vector<box3D>& box_3D);
    void check_status(vector<box3D>& box_3D);
};

class UVdetector
{
public:
    cv::Mat depth;
    cv::Mat depth_show;

    cv::Mat RGB;
    cv::Mat depth_low_res;
    cv::Mat U_map;
    cv::Mat U_map_show;
    int min_dist;
    int max_dist;
    int row_downsample;
    float col_scale;
    float threshold_point;
    float threshold_line;
    int min_length_line;
    bool show_bounding_box_U;
    std::vector<cv::Rect> bounding_box_U;
    std::vector<cv::Rect> bounding_box_B;
    std::vector<cv::Rect> bounding_box_D;
    std::vector<box3D> box3Ds;
    std::vector<box3D> box3DsWorld;

    int x0;
    int y0;

    int testx;
    int testy;
    int testby;

    float fx;
    float fy;
    float px;
    float py;
    double depthScale_;
    cv::Mat bird_view;
    UVtracker tracker;

    UVdetector();

    void readdata(queue<cv::Mat> depthq);
    void readdepth(cv::Mat depth);
    void readrgb(cv::Mat RGB);
    void extract_U_map();
    void extract_bb();
    void extract_bird_view();
    void detect();
    void track();
    void output();
    void display_depth();
    void extract_3Dbox();
    void display_U_map();
    void add_tracking_result();
    void display_bird_view();
};

UVbox merge_two_UVbox(UVbox father, UVbox son);

}  // namespace onboardDetector

#endif  // LVDOT_CORE_UV_DETECTOR_HPP
