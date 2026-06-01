#ifndef DEXHAND_UTILS_H
#define DEXHAND_UTILS_H
#include <string>
#include <vector>
#include "dexhand_def.h"

namespace eef_controller {
namespace dexhand_utils {

/**
 * @brief 解析手势文件
 * 
 * @param gesture_file_path 
 * @param gesture_infos 
 * @return true 
 * @return false 
 */
bool ParseGestureInfos(const std::string & gesture_file_path, std::vector<GestureInfoPtr> &gesture_infos);

} // namepspace dexhand_utils
} // namespace eef_controller

#endif // DEXHAND_UTILS_H

