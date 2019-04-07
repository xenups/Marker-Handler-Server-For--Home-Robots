FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/markerHandlerServer/msg"
  "src/markerHandlerServer/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/markerHandlerServer/get_marker.h"
  "srv_gen/cpp/include/markerHandlerServer/get_markers.h"
  "srv_gen/cpp/include/markerHandlerServer/set_marker.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
