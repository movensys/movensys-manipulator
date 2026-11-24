# curobo_node planner state for the RViz cuRobo panel (/curobo/planner_status, latched).
uint8 LEVEL_INFO = 0
uint8 LEVEL_OK = 1
uint8 LEVEL_BUSY = 2
uint8 LEVEL_ERROR = 3

string text
uint8 level

string[] start_states    # selectable start states, "<current>" first
string[] goal_states     # selectable goal states, "<marker>" and "<current>" first
string start_state
string goal_state

bool goal_ok             # goal has an IK solution
bool planning            # a preview plan is being computed
bool has_plan            # a previewed plan is ready
bool can_execute         # the previewed plan starts at the current robot state
bool moving              # executing a goal or the MPC is active
bool live_follow
float64 plan_duration    # s, previewed plan
