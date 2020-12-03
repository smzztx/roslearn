#include <fstream>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <assert.h>

int main()
{
	std::ifstream file_in("mymap.yaml");
	YAML::Node node = YAML::Load(file_in);//读取来自test.yaml的node文件
	std::cout << node["image"] <<std::endl;
	node["image"] = "/home/amy/amyos/mymap.pgm";

	std::ofstream file_out("obstable.yaml");
	file_out << node <<std::endl;

	return 0;
}