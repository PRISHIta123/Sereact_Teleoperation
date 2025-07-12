from moveit_commander import MoveGroupCommander
import threading

class MoveGroupInterface:
    def __init__(self, group_name, node):
        import moveit_commander
        moveit_commander.roscpp_initialize([])
        self.node = node
        self.lock = threading.Lock()
        self.group = MoveGroupCommander(group_name)

    def go_to_pose(self, pose_stamped):
        with self.lock:
            self.group.set_pose_target(pose_stamped)
            plan = self.group.plan()
            if plan and plan[0]:
                self.group.go(wait=True)
