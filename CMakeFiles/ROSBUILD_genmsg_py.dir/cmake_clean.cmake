FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/markerHandlerServer/msg"
  "src/markerHandlerServer/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/markerHandlerServer/msg/__init__.py"
  "src/markerHandlerServer/msg/_environment_marker.py"
  "src/markerHandlerServer/msg/_environment_markers.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
