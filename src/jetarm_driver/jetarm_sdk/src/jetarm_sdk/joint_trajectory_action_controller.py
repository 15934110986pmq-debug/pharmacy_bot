from threading import Thread

from math import radians
import rospy
import actionlib
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult


class Segment:
    def __init__(self, num_joints):
        self.start_time = 0.0  # trajectory segment start time
        self.duration = 0.0  # trajectory segment duration
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints


class JointTrajectoryActionController:
    def __init__(self, hw_interface):
        self.update_rate = 200
        self.state_update_rate = 20
        self.trajectory = []
        self.hw_interface = hw_interface

        self.stopped_velocity_tolerance = rospy.get_param('~stopped_velocity_tolerance', 0.01)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.goal_constraints = []
        self.trajectory_constraints = []
        self.min_velocity = 0.1

        for joint in self.joint_names:
            self.goal_constraints.append(-1.0)
            self.trajectory_constraints.append(-1.0)

        # Message containing current state for all controlled joints
        self.msg = FollowJointTrajectoryFeedback()
        self.msg.joint_names = self.joint_names
        self.msg.desired.positions = [0.0] *  len(self.joint_names)
        self.msg.desired.velocities = [0.0] * len(self.joint_names)
        self.msg.desired.accelerations = [0.0] * len(self.joint_names)
        self.msg.actual.positions = [0.0] * len(self.joint_names)
        self.msg.actual.velocities = [0.0] * len(self.joint_names)
        self.msg.error.positions = [0.0] * len(self.joint_names)
        self.msg.error.velocities = [0.0] * len(self.joint_names)

        self.running = True
        self.command_sub = rospy.Subscriber('/arm_controller/command', JointTrajectory, self.process_command)
        self.state_pub = rospy.Publisher('/arm_controller/state', FollowJointTrajectoryFeedback, queue_size=1)
        self.action_server = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory',
                                                          FollowJointTrajectoryAction,
                                                          execute_cb=self.process_follow_trajectory,
                                                          auto_start=False)
        self.action_server.start()

    def stop(self):
        self.running = False

    def process_command(self, msg):
        if self.action_server.is_active(): self.action_server.set_preempted()

        while self.action_server.is_active():
            rospy.sleep(0.01)

        self.process_trajectory(msg)

    def process_follow_trajectory(self, goal):
        self.process_trajectory(goal.trajectory)

    def process_trajectory(self, traj):
        num_points = len(traj.points)

        if num_points == 0:
            msg = 'Incoming trajectory is empty'
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return

        # correlate the joints we're commanding to the joints in the message
        # map from an index of joint in the controller to an index in the trajectory
        lookup = [traj.joint_names.index(joint) for joint in self.joint_names]
        durations = [0.0] * num_points

        # find out the duration of each segment in the trajectory
        durations[0] = traj.points[0].time_from_start.to_sec()
        for i in range(1, num_points):
            durations[i] = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).to_sec()

        if not traj.points[0].positions:
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            msg = 'First point of trajectory has no positions'
            rospy.logerr(msg)
            self.action_server.set_aborted(result=res, text=msg)
            return

        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)

        for i in range(num_points):
            seg = Segment(len(traj.joint_names))

            if traj.header.stamp == rospy.Time(0.0):
                seg.start_time = (time + traj.points[i].time_from_start).to_sec() - durations[i]
            else:
                seg.start_time = (traj.header.stamp + traj.points[i].time_from_start).to_sec() - durations[i]

            seg.duration = durations[i]

            # Checks that the incoming segment has the right number of elements.
            # if traj.points[i].velocities and len(traj.points[i].velocities) != self.num_joints:
            #     res = FollowJointTrajectoryResult()
            #     res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            #     msg = 'Command point %d has %d elements for the velocities' % (i, len(traj.points[i].velocities))
            #     rospy.logerr(msg)
            #     self.action_server.set_aborted(result=res, text=msg)
            #     return
            #
            # if len(traj.points[i].positions) != self.num_joints:
            #     res = FollowJointTrajectoryResult()
            #     res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            #     msg = 'Command point %d has %d elements for the positions' % (i, len(traj.points[i].positions))
            #     rospy.logerr(msg)
            #     self.action_server.set_aborted(result=res, text=msg)
            #     return

            for j in range(len(traj.joint_names)):
                if traj.points[i].positions:
                    seg.positions[j] = traj.points[i].positions[lookup[j]]

            trajectory.append(seg)

        rospy.loginfo('Trajectory start requested at %.3lf, waiting...', traj.header.stamp.to_sec())
        rate = rospy.Rate(self.update_rate)

        while traj.header.stamp > time:
            time = rospy.Time.now()
            rate.sleep()

        end_time = traj.header.stamp + rospy.Duration(sum(durations))
        seg_end_times = [rospy.Time.from_sec(trajectory[seg].start_time + durations[seg]) for seg in
                         range(len(trajectory))]

        rospy.loginfo('Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf', time.to_sec(),
                      end_time.to_sec(), sum(durations))

        self.trajectory = trajectory

        for seg in range(len(trajectory)):
            rospy.logdebug('current segment is %d time left %f cur time %f' % (
                seg, durations[seg] - (time.to_sec() - trajectory[seg].start_time), time.to_sec()))
            rospy.logdebug('goal positions are: %s' % str(trajectory[seg].positions))

            # first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
            if durations[seg] == 0:
                rospy.logdebug('skipping segment %d with duration of 0 seconds' % seg)
                continue

            for i in range(len(traj.joint_names)):
                desired_position = trajectory[seg].positions[i]
                self.msg.desired.positions[i] = desired_position
                self.hw_interface(traj.joint_names[i], desired_position, int(durations[seg] * 1000 + 0.5))

            while time < seg_end_times[seg]:
                # heck if new trajectory was received, if so abort current trajectory execution
                # by setting the goal to the current position c
                if self.action_server.is_preempt_requested():
                    msg = 'New trajectory received. Exiting.'
                    self.action_server.set_preempted(text=msg)
                    rospy.loginfo(msg)
                    return

                rate.sleep()
                time = rospy.Time.now()

            # Verifies trajectory constraints
            #for j, joint in enumerate(traj.joint_names):
            #    if self.trajectory_constraints[j] > 0 and self.msg.error.positions[j] > self.trajectory_constraints[j]:
            #        res = FollowJointTrajectoryResult()
            #        res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            #        msg = 'Unsatisfied position constraint for %s, trajectory point %d, %f is larger than %f' % \
            #              (joint, seg, self.msg.error.positions[j], self.trajectory_constraints[j])
            #        rospy.logwarn(msg)
            #        self.action_server.set_aborted(result=res, text=msg)
            #        return

        # Checks that we have ended inside the goal constraints
        for (joint, pos_error, pos_constraint) in zip(traj.joint_names, self.msg.error.positions,
                                                      self.goal_constraints):
            if pos_constraint > 0 and abs(pos_error) > pos_constraint:
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                msg = 'Aborting because %s joint wound up outside the goal constraints, %f is larger than %f' % \
                      (joint, pos_error, pos_constraint)
                rospy.logwarn(msg)
                self.action_server.set_aborted(result=res, text=msg)
                break
        else:
            msg = 'Trajectory execution successfully completed'
            rospy.loginfo(msg)
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.action_server.set_succeeded(result=res, text=msg)

