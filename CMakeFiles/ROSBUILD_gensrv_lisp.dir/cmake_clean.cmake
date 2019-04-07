FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/markerHandlerServer/msg"
  "src/markerHandlerServer/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/get_marker.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_get_marker.lisp"
  "srv_gen/lisp/get_markers.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_get_markers.lisp"
  "srv_gen/lisp/set_marker.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_set_marker.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
