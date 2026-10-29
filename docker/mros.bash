MOVENSYS_MANIPULATOR_CONTAINER="${MOVENSYS_MANIPULATOR_CONTAINER:-movensys_manipulator_container}"
MOVENSYS_MANIPULATOR_WORKSPACE="${MOVENSYS_MANIPULATOR_WORKSPACE:-/home/admin/workspaces/movensys_ws}"

mros() {
  local flags=(-i)
  [ -t 0 ] && flags=(-it)

  local setup='source /opt/ros/${ROS_DISTRO}/setup.bash'
  setup="${setup} && source ${MOVENSYS_MANIPULATOR_WORKSPACE}/install/setup.bash"

  if [ $# -eq 0 ]; then
    docker exec "${flags[@]}" -u admin "${MOVENSYS_MANIPULATOR_CONTAINER}" \
      bash -lc "${setup} && exec bash -i"
  else
    docker exec "${flags[@]}" -u admin "${MOVENSYS_MANIPULATOR_CONTAINER}" \
      bash -lc "${setup} && $*"
  fi
}
