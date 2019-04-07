FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/markerHandlerServer/msg"
  "src/markerHandlerServer/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/markerHandlerServer/srv/__init__.py"
  "src/markerHandlerServer/srv/_get_marker.py"
  "src/markerHandlerServer/srv/_get_markers.py"
  "src/markerHandlerServer/srv/_set_marker.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
