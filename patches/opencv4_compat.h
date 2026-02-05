/**
 * OpenCV 4.x 兼容性头文件
 * 
 * 策略：
 * 1. 先 include OpenCV 4.x 保留的 C 风格兼容头文件，获取大部分旧常量
 * 2. 只定义那些被完全移除的常量
 * 
 * 用于修复 VINS-Fusion 在 OpenCV 4.x 下的编译错误。
 * 
 * ============================================================
 * 使用方法 (使用绝对路径引用，无需复制文件)
 * ============================================================
 * 
 * 1. 在报错的 VINS-Fusion 源文件开头添加 include (使用绝对路径):
 * 
 *    #include "/home/amov/Projects/patches/opencv4_compat.h"
 * 
 * 2. 需要添加此 include 的文件列表（已添加）:
 *    - vins_estimator/src/featureTracker/feature_tracker.cpp 
 *    - camera_models/src/chessboard/Chessboard.cc
 *    - camera_models/src/calib/CameraCalibration.cc
 *    - camera_models/src/intrinsic_calib.cc
 *    - loop_fusion/src/pose_graph.cpp
 *    - loop_fusion/src/keyframe.cpp
 *    - loop_fusion/src/ThirdParty/DVision/BRIEF.cpp
 *    - vins_estimator/src/KITTIGPSTest.cpp
 *    - vins_estimator/src/KITTIOdomTest.cpp
 * 
 * 3. 重新编译 catkin 工作空间:
 *    cd /home/amov/Projects/catkin_ws && catkin_make
 * 
 * 4. 如需还原 VINS-Fusion 源码:
 *    cd /home/amov/Projects/catkin_ws/src/VINS-Fusion && git checkout .
 * 
 * ============================================================
 */

#ifndef OPENCV4_COMPAT_H
#define OPENCV4_COMPAT_H

#include <opencv2/core/version.hpp>

#if CV_VERSION_MAJOR >= 4

// ==========================================
// 包含 OpenCV 4.x 保留的 C 风格 API 头文件
// 这些文件定义了旧的 CV_BGR2GRAY, CV_THRESH_BINARY 等常量
// ==========================================
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/types_c.h>

// ==========================================
// 以下常量在 OpenCV 4.x 中被完全移除，需要手动定义
// ==========================================

// 绘图常量 - CV_AA 被移除
#ifndef CV_AA
#define CV_AA cv::LINE_AA
#endif

#ifndef CV_FILLED
#define CV_FILLED cv::FILLED
#endif

// ==========================================
// 棋盘检测标志 - 这些从 CV_ 前缀改为 cv:: 命名空间
// ==========================================
#ifndef CV_CALIB_CB_ADAPTIVE_THRESH
#define CV_CALIB_CB_ADAPTIVE_THRESH cv::CALIB_CB_ADAPTIVE_THRESH
#endif

#ifndef CV_CALIB_CB_NORMALIZE_IMAGE
#define CV_CALIB_CB_NORMALIZE_IMAGE cv::CALIB_CB_NORMALIZE_IMAGE
#endif

#ifndef CV_CALIB_CB_FILTER_QUADS
#define CV_CALIB_CB_FILTER_QUADS cv::CALIB_CB_FILTER_QUADS
#endif

#ifndef CV_CALIB_CB_FAST_CHECK
#define CV_CALIB_CB_FAST_CHECK cv::CALIB_CB_FAST_CHECK
#endif

// ==========================================
// 终止条件常量
// ==========================================
#ifndef CV_TERMCRIT_EPS
#define CV_TERMCRIT_EPS cv::TermCriteria::EPS
#endif

#ifndef CV_TERMCRIT_ITER
#define CV_TERMCRIT_ITER cv::TermCriteria::MAX_ITER
#endif

#ifndef CV_TERMCRIT_NUMBER
#define CV_TERMCRIT_NUMBER cv::TermCriteria::COUNT
#endif

// ==========================================
// 字体常量 (如果需要)
// ==========================================
#ifndef CV_FONT_HERSHEY_SIMPLEX
#define CV_FONT_HERSHEY_SIMPLEX cv::FONT_HERSHEY_SIMPLEX
#endif

#ifndef CV_FONT_HERSHEY_PLAIN
#define CV_FONT_HERSHEY_PLAIN cv::FONT_HERSHEY_PLAIN
#endif

// ==========================================
// 图像读取标志 (imread)
// ==========================================
#ifndef CV_LOAD_IMAGE_GRAYSCALE
#define CV_LOAD_IMAGE_GRAYSCALE cv::IMREAD_GRAYSCALE
#endif

#ifndef CV_LOAD_IMAGE_COLOR
#define CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
#endif

#ifndef CV_LOAD_IMAGE_UNCHANGED
#define CV_LOAD_IMAGE_UNCHANGED cv::IMREAD_UNCHANGED
#endif

#endif // CV_VERSION_MAJOR >= 4

#endif // OPENCV4_COMPAT_H
