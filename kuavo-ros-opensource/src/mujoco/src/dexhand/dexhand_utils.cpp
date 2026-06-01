#include "dexhand_utils.h"
#include "json.hpp"
#include <iostream>
#include <fstream>

namespace eef_controller {
namespace dexhand_utils {


bool ParseGestureInfos(const std::string & gesture_file_path, std::vector<GestureInfoPtr> &gesture_infos)
{
    nlohmann::json data;
    try {
        std::ifstream file(gesture_file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to gesture config file: " << gesture_file_path << std::endl;
            return false;
        }

        file >> data;    /* file to json data */

        for(auto &item : data) {
            GestureInfoPtr gest_info = std::make_shared<GestureInfo>();
            gest_info->gesture_name = item["gesture_name"];
            gest_info->description = item["description"];

            /* field action_sequences */
            const auto &action_sequences = item["action_sequences"];
            for(auto &as : action_sequences) {
                ActionSequenceData as_data;
                as_data.duration_ms = as["duration_ms"];
                if(as["positions"].is_array() && as["positions"].size() == 6) {
                    // std::cout << "pos: ";
                    for(int i = 0; i < 6; i++) {
                        as_data.positions[i] = as["positions"][i];
                        // std::cout << " " << as["positions"][i];
                    }
                    // std::cout << std::endl;
                }
                if(as["speeds"].is_array() && as["speeds"].size() == 6) {
                    for(int i = 0; i < 6; i++) {
                        as_data.speeds[i] = as["speeds"][i];
                    }
                }
                if(as["forces"].is_array() && as["forces"].size() == 6) {
                    for(int i = 0; i < 6; i++) {
                        as_data.forces[i] = as["forces"][i];
                    }
                }
                gest_info->action_sequences.emplace_back(as_data);
            }

            /* field alias */
            const auto &alias = item["alias"];
            for(auto &alias_item : alias) {
                gest_info->alias.emplace_back(alias_item);
            }
            
            /* print gesture */
            // std::cout << "@@@ gesture: " << gest_info->gesture_name << ", ";
            // for (const auto& alias_name : gest_info->alias)   {
            //     std::cout << alias_name << ", "; 
            // }
            // std::cout << "\n";

            gesture_infos.emplace_back(gest_info);
        }
    }
    catch(std::exception& e) {
        std::cout << " parse gesture file failed, e.what(): " << e.what() << "\n";
        return false;
    }
            
    return true;
}

} // namespace dexhand_utils
} // namespace eef_controller