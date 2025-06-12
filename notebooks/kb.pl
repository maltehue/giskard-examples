% Knowledge Base

gripper(pr2, l_gripper_tool_frame).
gripper(pr2, r_gripper_tool_frame).

openState(fridge, 0.8).
openState(drawer_01, 0.2).

closeState(fridge, 0).
closeState(drawer_01, 0).

robot(pr2).

state(fridge , closed).
state(drawer_01, closed).

motion(envJointGoal).

motion_effect(envJointGoal, open(Container)) :- container(Container), openState(Container, _).
motion_effect(envJointGoal, close(Container)) :- container(Container), closeState(Container, _).

%canExecute_envJointGoal(Motion, Joint, GoalState, Handle, Gripper) :- execute_motion_envJointGoal(Motion, Joint, GoalState, Handle, Gripper).
canExecute_envJointGoal(Motion, Joint, GoalState, Handle, Gripper) :- add_init_pose(), execute(),
                                                                                add_open_container(Motion, Joint,
                                                                                GoalState, Handle, Gripper),
                                                                                giskard_project_eval().

action_desig(open_fridge, [perform, [action, [type, open], [object_acted_on, fridge]]]).
action_desig(close_anything, [perform, [action, [type, close], [object_acted_on, Container]]]) :- container(Container).
action_desig(open_anything, [perform, [action, [type, open], [object_acted_on, Container]]]) :- container(Container).

holds(open(Container), task_end([perform, [action, [type, open], [object_acted_on, Container]]])).
holds(close(Container), task_end([perform, [action, [type, close], [object_acted_on, Container]]])).

environmentForStateChange(open(Container), Handle, Joint, GoalState) :- container(Container),
                                                                        container_articulation(Container, Handle, Joint)
                                                                        openState(Container, GoalState),
                                                                        state(Container, closed).
environmentForStateChange(close(Container), Handle, Joint, GoalState) :- container(Container),
                                                                        container_articulation(Container, Handle, Joint)
                                                                        closeState(Container, GoalState),
                                                                        state(Container, opened).

taskRequest(Request, StateChange) :- action_desig(Request, Action),
                                     Action = [perform, [action, [type, ActionType], [object_acted_on, Container]]],
                                     holds(StateChange, task_end(Action)),
                                     StateChange =.. [ActionType, Container].

causes(Motion, StateChange, MotionParam) :- motion_effect(Motion, StateChange),
                                            (
                                                StateChange = open(Container) -> environmentForStateChange(open(Container), Handle, Joint, GoalState);
                                                StateChange = close(Container) -> environmentForStateChange(close(Container), Handle, Joint, GoalState)
                                            ),
                                            (
                                                Motion == envJointGoal -> MotionParam = [GoalState, Joint, Handle]
                                            ),
                                            traj_open_container(Motion, Joint, GoalState, Handle).
canPerform(Robot, Motion, MotionParam) :- robot(Robot),
                                          gripper(Robot, Gripper),
                                          (
                                                Motion == envJointGoal -> MotionParam = [GoalState, Joint, Handle]
                                          ),
                                          canExecute_envJointGoal(Motion, Joint, GoalState, Handle, Gripper).
%traj_open_container(Motion, Joint, GoalState, Handle), execute().

