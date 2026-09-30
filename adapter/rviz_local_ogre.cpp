// Relocate Debian RViz's compiled-in OGRE directory without system changes.
// Loaded only by run_rviz.sh; the estimator does not use this library.
#include <cstdlib>
#include <stdexcept>
#include <string>
namespace rviz {
std::string get_ogre_plugin_path() {
  const char *root=std::getenv("VIO_ROOT");
  if(!root) throw std::runtime_error("VIO_ROOT is required for local RViz");
  return std::string(root)+"/deps/usr/lib/aarch64-linux-gnu/OGRE";
}
}
