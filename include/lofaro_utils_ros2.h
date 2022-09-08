// String Split
#include <iostream>
#include <vector>
#include <sstream>
#include <string>


namespace lofaro_utils_ros2 
{
  void do_split(std::string const &str, const char delim,
    std::vector<std::string> &out)
  {
   // construct a stream from the string
   std::stringstream ss(str);

   std::string s;
   while (std::getline(ss, s, delim)) {
       out.push_back(s);
   }
  }
}


