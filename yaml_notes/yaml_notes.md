# yaml_notes

> 参考博客：https://blog.csdn.net/u014610460/article/details/79508869



1. yaml的读取

```cpp
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
 
int main(int argc, char **argv)
{
    std::string fin = "/home/user/param/param.yaml";       //yaml文件所在的路径
    YAML::Node yamlConfig = YAML::LoadFile(fin);
    int int_param = yamlConfig["int_param"].as<int>();
    std::cout << "  node size: " << yamlConfig.size() << std::endl;
    std::cout << yamlConfig["bool_param"].as<bool>() << "\n";
    yamlConfig["bool_param"] = !yamlConfig["bool_param"].as<bool>();
    yamlConfig["str_param"] = "test";
    std::ofstream file;
    file.open(fin);
    file.flush();
    file << yamlConfig;
    file.close();
 
    return 0;
}
```

cmake配置

```cmake
FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)
  # If yaml-cpp not found in the system, try finding it as a user CMake-generated project
  FIND_PACKAGE(yaml-cpp REQUIRED)
  INCLUDE_DIRECTORIES(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)

link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)

target_link_libraries(yaml_test ${YAML_CPP_LIBRARIES})
```



2. yaml文件生成

```cpp
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
 
int main(int argc, char **argv)
{
    std::ofstream fout("/home/user/param/param.yaml");
    YAML::Emitter out(fout);
    out << YAML::BeginMap;
    out << YAML::Key << "int_param";
    out << YAML::Value << 1;
    out << YAML::Key << "double_param";
    out << YAML::Value << 0.5;
    out << YAML::Key << "bool_param";
    out << YAML::Value << false;
    out << YAML::Comment("bool parameter");
    out << YAML::Key << "str_param";
    out << YAML::Value << "test";
    out << YAML::EndMap;
    
    return 0;
}
```