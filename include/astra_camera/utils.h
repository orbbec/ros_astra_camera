/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/
#pragma once

#include <openni2/OpenNI.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointCloud2.h>

#include "astra_camera/Extrinsics.h"
#include "constants.h"
#include "types.h"


namespace astra_camera {
    inline void LogFatal(const char *file, int line, const std::string &message) {
        std::cerr << "Check failed at " << file << ":" << line << ": " << message << std::endl;
        std::abort();
    }
}  // namespace orbbec_camera

// Macros for checking conditions and comparing values
#define CHECK(condition) \
  (!(condition) ? LogFatal(__FILE__, __LINE__, "Check failed: " #condition) : (void)0)

template<typename T1, typename T2>
void CheckOp(const char *expr, const char *file, int line, T1 val1, T2 val2, bool result) {
    if (!result) {
        std::ostringstream os;
        os << "Check failed: " << expr << " (" << val1 << " vs. " << val2 << ")";
        astra_camera::LogFatal(file, line, os.str());
    }
}

#define CHECK_OP(opname, op, val1, val2) \
  CheckOp(#val1 " " #op " " #val2, __FILE__, __LINE__, val1, val2, (val1)op(val2))

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, <, val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, >, val1, val2)

// Overload for raw pointers
template<typename T>
T *CheckNotNull(T *ptr, const char *file, int line) {
    if (ptr == nullptr) {
        std::ostringstream os;
        os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
        astra_camera::LogFatal(file, line, os.str());
    }
    return ptr;
}

// Template for smart pointers like std::shared_ptr, std::unique_ptr
template<typename T>
T &CheckNotNull(T &ptr, const char *file, int line) {
    if (ptr == nullptr) {
        std::ostringstream os;
        os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
        astra_camera::LogFatal(file, line, os.str());
    }
    return ptr;
}

#if defined(CHECK_NOTNULL)
#undef CHECK_NOTNULL
#endif
#define CHECK_NOTNULL(val) CheckNotNull(val, __FILE__, __LINE__)

namespace astra_camera {
    bool operator==(const openni::VideoMode &lhs, const openni::VideoMode &rhs);

    bool operator!=(const openni::VideoMode &lhs, const openni::VideoMode &rhs);

    std::ostream &operator<<(std::ostream &os, const openni::VideoMode &video_mode);

    tf2::Quaternion rotationMatrixToQuaternion(const std::vector<float> &rotation);

    Extrinsics obExtrinsicsToMsg(const std::vector<float> &rotation, const std::vector<float> &transition,
                                 const std::string &frame_id);


    bool isValidCameraParams(const OBCameraParams &params);

    void cameraParameterPrinter(const std::vector<float> &rotation,
                                const std::vector<float> &transition);

    std::string PixelFormatToString(const openni::PixelFormat &format);

    void savePointToPly(sensor_msgs::PointCloud2::Ptr cloud, const std::string &filename);

    void saveRGBPointToPly(sensor_msgs::PointCloud2::Ptr cloud, const std::string &filename);

    MultiDeviceSyncMode getMultiDeviceSyncMode(const std::string &mode);


}  // namespace astra_camera
