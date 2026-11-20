#!/usr/bin/env bash
# Run the same PF, planner, controller, command mux, or observer on the Isaac host.
set -e
workspace="${DAMVI_SERVER_ROS_WS:-$HOME/Desktop/RC_Car_Isaac/headless_ros}"
project="$workspace/project"
drive="$project/driving_validation"
source /opt/ros/jazzy/setup.bash
source "$HOME/damvi_vesc_ros_ws/install/local_setup.bash"
if [[ -f "$HOME/damvi_headless_msgs/install/local_setup.bash" ]]; then
  source "$HOME/damvi_headless_msgs/install/local_setup.bash"
fi
source "$workspace/install/local_setup.bash"
export LD_LIBRARY_PATH="$workspace/deps/opt/ros/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export DAMVI_SERVER_ROS_WS="$workspace"
export DAMVI_DATASET_ROOT="$workspace/datasets"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" PYTHONNOUSERSITE=1
action="$1"
shift
case "$action" in
  pf)
    dataset="$1"
    readarray -t map_files < <(python3 - "$DAMVI_DATASET_ROOT/$dataset" <<'PY'
import pathlib,sys
p=pathlib.Path(sys.argv[1]);print(*(f.stem for f in p.glob('*.yaml') if f.name!='metadata.yaml'),sep='\n')
PY
)
    [[ ${#map_files[@]} == 1 && -n "${map_files[0]}" ]]
    exec ros2 run localization_pf localization_pf_node --ros-args \
      --params-file "$project/localization_validation/config/sim0821.yaml" \
      --params-file "$drive/config/headless_pf_precompute.yaml" \
      -p "map_loader.map_dir:=$DAMVI_DATASET_ROOT/$dataset" -p "map_loader.map_name:=${map_files[0]}"
    ;;
  stack)
    export DAMVI_HEADLESS_DATASET="$1" DAMVI_SPEED_CAP="$2"
    case "$3" in
      tuned) export DAMVI_PP_OVERLAY="$drive/config/pp_tracking_tuned.yaml" ;;
      preview) export DAMVI_PP_OVERLAY="$drive/config/pp_tracking_preview.yaml" ;;
      baseline) export DAMVI_PP_OVERLAY="" ;;
      *) if [[ -f "$3" ]]; then
           export DAMVI_PP_OVERLAY="$(realpath -- "$3")"
         else
           echo "Choose baseline, tuned, preview, or an existing custom PP YAML" >&2; exit 2
         fi ;;
    esac
    exec ros2 launch "$drive/launch/driving_stack.launch.py"
    ;;
  lap) exec /usr/bin/python3 "$drive/tools/run_headless_lap.py" "$@" --same-host ;;
  seed) exec /usr/bin/python3 "$drive/tools/seed_headless_stack.py" "$@" ;;
  observe) exec /usr/bin/python3 "$drive/tools/observe_lap.py" "$@" ;;
  score) exec /usr/bin/python3 "$drive/tools/score_lap.py" "$@" ;;
  stop) exec timeout 10 ros2 service call /validation/enable_autonomy std_srvs/srv/SetBool '{data: false}' ;;
  *) echo "Usage: $0 pf DATASET | stack DATASET CAP baseline|tuned | seed DATASET | observe ... | score ... | stop" >&2; exit 2 ;;
esac
