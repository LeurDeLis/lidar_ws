/**
 * @file lipkg.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lipkg.h"
#include "ros_api.h"

namespace ldlidar {

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiPkg::LiPkg() : ld_product_type_(LDType::NO_VER),
  sdk_pack_verison_("v2.2.1"),
  laser_scan_dir_(false),
  is_frame_ready_(false),
  timestamp_(0),
  speed_(0),
  point_frequence_(2300) {
  
}

LiPkg::~LiPkg() {

}

bool LiPkg::AnalysisOne(uint8_t byte) {
  static enum {
    HEADER,
    VER_LEN,
    DATA,
  } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};
  static uint16_t pkg_count = sizeof(LiDARFrameTypeDef);

  switch (state) {
    case HEADER:
      if (byte == PKG_HEADER) {
        tmp[count++] = byte;
        state = VER_LEN;
      }
      break;
    case VER_LEN:
      if (byte == PKG_VER_LEN) {
        tmp[count++] = byte;
        state = DATA;
      } else {
        state = HEADER;
        count = 0;
        return false;
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        memcpy((uint8_t *)&pkg, tmp, pkg_count);
        uint8_t crc = CalCRC8((uint8_t *)&pkg, pkg_count - 1);
        state = HEADER;
        count = 0;
        if (crc == pkg.crc8) {
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false;  
}

bool LiPkg::Parse(const uint8_t *data, long len) {
  // 遍历接收到的字节数据
  for (int i = 0; i < len; i++) {
    // 调用 AnalysisOne 方法解析单个字节
    if (AnalysisOne(data[i])) {
      // 如果解析成功，说明一个完整的数据包已经解析完成
      // 计算起始角度和结束角度之间的差值（以度为单位）
      double diff = (pkg.end_angle / 100 - pkg.start_angle / 100 + 360) % 360;
      // ROS_INFO("pkg.speed = %d", pkg.speed);
      // 检查角度差是否在合理范围内
      // 条件：角度差应小于等于速度、点数和频率的函数值乘以 1.5 倍
      if (diff <= ((double)pkg.speed * POINT_PER_PACK / point_frequence_ * 1.6)) {
        // ROS_INFO("-+---+-+-+-+-++--+-+");
        // 更新速度和时间戳
        speed_ = pkg.speed;  // 更新当前扫描速度
        timestamp_ = pkg.timestamp;  // 更新时间戳
        // 计算起始角度和结束角度之间的差值（以整数值表示，单位为 0.01 度）
        uint32_t diff = ((uint32_t)pkg.end_angle + 36000 - (uint32_t)pkg.start_angle) % 36000;
        // 计算每个点之间的角度步长（单位为度）
        float step = diff / (POINT_PER_PACK - 1) / 100.0;
        // 获取起始角度（单位为度）
        float start = (double)pkg.start_angle / 100.0;
        // 定义一个临时变量存储点数据
        PointData data;
        // 遍历数据包中的所有点（假设每个包包含 POINT_PER_PACK 个点）
        for (int i = 0; i < POINT_PER_PACK; i++) {
          // 设置点的距离
          data.distance = pkg.point[i].distance;
          // 计算点的角度
          data.angle = start + i * step;
          // 如果角度超过 360 度，则将其归一化到 [0, 360) 范围内
          if (data.angle >= 360.0) {
            data.angle -= 360.0;
          }
          // 设置点的强度
          data.intensity = pkg.point[i].intensity;
          // 将点数据添加到临时帧缓冲区中
          frame_tmp_.push_back(PointData(data.angle, data.distance, data.intensity));
        }
      }
    }
  }
  // 返回 true 表示解析过程完成
  return true;
}

bool LiPkg::AssemblePacket() {
  float last_angle = 0;  // 记录上一个点的角度，用于判断是否完成一圈扫描
  Points2D tmp, data;    // 定义两个点数据容器：tmp 用于存储最终处理后的点数据，data 用于存储临时数据
  int count = 0;         // 计数器，记录当前处理的点数量

  // 如果速度小于等于 0，说明数据无效，清空缓冲区并返回 false
  if (speed_ <= 0) {
    frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.end());  // 清空临时帧缓冲区
    return false;
  }

  // 遍历临时帧缓冲区中的所有点数据
  for (auto n : frame_tmp_) {
    // 检查是否已经收集到足够的数据以完成一圈扫描
    // 条件：当前点的角度小于 20 度，且上一个点的角度大于 340 度
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      // 检查数据点数量是否合理（防止数据过多或过少）
      // 条件：点数量乘以速度应小于等于频率的 1.4 倍
      if ((count * GetSpeed()) > (point_frequence_ * 1.4)) {
        // 如果点数量超出范围，清空或删除多余的数据
        if (count >= (int)frame_tmp_.size()) {
          frame_tmp_.clear();  // 如果计数器超过缓冲区大小，清空整个缓冲区
        } else {
          frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);  // 删除已处理的点
        }
        return false;  // 返回 false 表示组装失败
      }

      // 将当前处理的点插入到临时数据容器中
      data.insert(data.begin(), frame_tmp_.begin(), frame_tmp_.begin() + count);

      // 坐标系转换：将原始数据从左手法则（逆时针）转换为右手法则（顺时针）
      SlTransform trans(ld_product_type_, laser_scan_dir_);
      data = trans.Transform(data);  // 转换数据为标准格式

      // 根据设备类型选择不同的滤波方式
      if (LDType::LD_14P == ld_product_type_) {
        tmp = data;  // 如果是 LD14P 类型，直接使用转换后的数据
      } else {
        Slbf sb(speed_);  // 创建滤波器对象
        tmp = sb.NearFilter(data);  // 对数据进行近邻滤波
      }

      // 对点数据按角度从小到大排序
      std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) { return a.angle < b.angle; });

      // 如果处理后的点数据不为空，则设置激光扫描数据并标记帧准备就绪
      if (tmp.size() > 0) {
        SetLaserScanData(tmp);  // 设置激光扫描数据
        // ROS_INFO("++++++++++++++++++++");  // 打印调试信息
        SetFrameReady();  // 标记帧准备就绪
        // ROS_INFO("--------------------");  // 打印调试信息

        // 清理已处理的数据
        if (count >= (int)frame_tmp_.size()) {
          frame_tmp_.clear();  // 如果计数器超过缓冲区大小，清空整个缓冲区
        } else {
          frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);  // 删除已处理的点
        }
        return true;  // 返回 true 表示组装成功
      }
    }
    // ROS_INFO("--------//////--------");  // 打印调试信息
    // 更新计数器和上一个点的角度
    count++;
    last_angle = n.angle;

    // 检查数据点数量是否过多（防止无限增长）
    // 条件：点数量乘以速度应小于等于频率的 2 倍
    if ((count * GetSpeed()) > (point_frequence_ * 2)) {
      // 如果点数量超出范围，清空或删除多余的数据
      if (count >= (int)frame_tmp_.size()) {
        frame_tmp_.clear();  // 如果计数器超过缓冲区大小，清空整个缓冲区
      } else {
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);  // 删除已处理的点
      }
      // ROS_INFO("--------+*+*+*---------");  // 打印调试信息
      return false;  // 返回 false 表示组装失败
    }
  }
// ROS_INFO("---------******-----------");  // 打印调试信息
  // 如果未完成一圈扫描，返回 false 表示组装失败
  return false;
}
void LiPkg::CommReadCallback(const char *byte, size_t len) {
  if (this->Parse((uint8_t *)byte, len)) {
    // ROS_INFO("Received %zu bytes of data", len);
    this->AssemblePacket();
  }
}

std::string LiPkg::GetSdkVersionNumber(void) {
  return sdk_pack_verison_;
}

void LiPkg::SetProductType(LDType type_number) {
  ld_product_type_ = type_number;
  switch (type_number) {
    case LDType::LD_14P:
      point_frequence_ = 4000;
      break;
    default:
      point_frequence_ = 2300;
      break;
  }
}

void LiPkg::SetLaserScanDir(bool dir) {
  laser_scan_dir_ = dir;
}

double LiPkg::GetSpeed(void) { 
  return (speed_ / 360.0);  // unit is hz 
}

uint16_t LiPkg::GetSpeedOrigin(void) {
  return speed_;
}

uint16_t LiPkg::GetTimestamp(void) {
  return timestamp_;
}

bool LiPkg::IsFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  return is_frame_ready_;
}


void LiPkg::ResetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = false;
}

void LiPkg::SetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = true;
}

Points2D LiPkg::GetLaserScanData(void) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  return laser_scan_data_;
}

void LiPkg::SetLaserScanData(Points2D& src) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  laser_scan_data_ = src;
}


} // namespace ldlidar 

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/