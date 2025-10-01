
#pragma once
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
class ModuleBase {
   private:
    YAML::Node config_node;
    std::string name;

   protected:
    /**
     * @brief Construct a new Module Base object
     *
     * @param config_path 配置文件目录
     * @param prefix //前缀
     * @param module_name //模块名字
     */
    ModuleBase(const std::string &config_path, const std::string &prefix, const std::string &module_name = " default") {
        name = module_name;
        if (config_path != "") {
            try {
                config_node = YAML::LoadFile(config_path);
            } catch (YAML::Exception &e) {
                std::cout << e.msg << std::endl;
            }
        }
        if (prefix != "" && config_node[prefix]) {
            config_node = config_node[prefix];
        }
    }
    template <typename T>
    /**
     * @brief
     *
     * @param key 键
     * @param val 值
     * @param default_val 默认值
     */
    void readParam(const std::string &key, T &val, T default_val) {
        if (config_node[key]) {
            val = config_node[key].as<T>();
        } else {
            val = default_val;
        }
    }
};