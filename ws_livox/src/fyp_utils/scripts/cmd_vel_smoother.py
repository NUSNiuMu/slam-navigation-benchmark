#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


class VelocitySmoother(object):
    def __init__(self):
        rospy.init_node("cmd_vel_smoother")

        self.accel_lim_v = rospy.get_param("~accel_lim_v", 0.15)
        self.accel_lim_w = rospy.get_param("~accel_lim_w", 0.40)
        self.decel_lim_v = rospy.get_param("~decel_lim_v", self.accel_lim_v * 2.0)
        self.decel_lim_w = rospy.get_param("~decel_lim_w", self.accel_lim_w * 2.0)
        self.hz = rospy.get_param("~hz", 50.0)
        self.timeout = rospy.get_param("~cmd_timeout", 0.5)
        self.hard_stop_on_timeout = rospy.get_param("~hard_stop_on_timeout", False)
        self.dt = 1.0 / self.hz

        self.current_vel = Twist()
        self.target_vel = Twist()
        self.last_cmd_time = rospy.Time.now()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.vel_cb, queue_size=10)
        rospy.Timer(rospy.Duration(self.dt), self.timer_cb)

    def vel_cb(self, msg):
        self.target_vel = msg
        self.last_cmd_time = rospy.Time.now()

    @staticmethod
    def ramp(current, target, accel_limit, decel_limit, dt):
        delta = target - current
        # Use a larger limit when reducing speed magnitude so stop feels more responsive.
        if abs(target) < abs(current):
            limit = decel_limit
        else:
            limit = accel_limit
        max_step = limit * dt
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def timer_cb(self, _):
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.timeout:
            self.target_vel = Twist()
            if self.hard_stop_on_timeout:
                self.current_vel = Twist()
                self.pub.publish(self.current_vel)
                return

        self.current_vel.linear.x = self.ramp(
            self.current_vel.linear.x,
            self.target_vel.linear.x,
            self.accel_lim_v,
            self.decel_lim_v,
            self.dt,
        )
        self.current_vel.angular.z = self.ramp(
            self.current_vel.angular.z,
            self.target_vel.angular.z,
            self.accel_lim_w,
            self.decel_lim_w,
            self.dt,
        )

        self.pub.publish(self.current_vel)


if __name__ == "__main__":
    try:
        VelocitySmoother()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
