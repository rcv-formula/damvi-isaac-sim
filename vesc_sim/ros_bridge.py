"""ROS I/O only. No speed is inferred from chassis motion here."""
import math
import time


class RosBridge:
    def __init__(self, config):
        import rclpy
        from std_msgs.msg import Float64
        from vesc_msgs.msg import VescStateStamped
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
        from rclpy.subscription import Subscription
        self.rclpy=rclpy;self.message_type=VescStateStamped;self.clock_type=Clock
        self.context=rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node=rclpy.create_node('damvi_vesc_sim',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.config=config;self.time=0.0;self.received=None
        self.target=0.0;self.brake=None;self.was_stopped=False
        self.steering_input=0.;self.steering_received=None
        self.callback_count=0
        self.publisher=self.node.create_publisher(VescStateStamped,config['ros']['output_topic'],10)
        self.clock=self.node.create_publisher(Clock,'/clock',10)
        # Only the current requested setpoint matters to the speed/steering controller.
        latest=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.RELIABLE)
        self.speed_subscription=self.node.create_subscription(Float64,config['ros']['command_topic'],self.command,latest)
        self.node.create_subscription(Float64,config['ros']['brake_topic'],self.brake_command,10)
        self.validation_command_receipts=None
        callback=self.stamped_cmd_vel if hasattr(Subscription,'CallbackType') else self.cmd_vel
        self.twist_subscription=self.node.create_subscription(Twist,config.get('_steering',{}).get('input_topic','/cmd_vel'),callback,latest)

    def stamped_cmd_vel(self, message, info):
        if self.validation_command_receipts is not None:
            def timestamp(key):
                return info.get(key,0) if isinstance(info,dict) else getattr(info,key,0)
            self.validation_command_receipts.append([self.time, time.time_ns(),
                int(timestamp('source_timestamp')), int(timestamp('received_timestamp')),
                float(message.linear.x), float(message.angular.z)])
        self.cmd_vel(message)

    def cmd_vel(self,m):
        self.callback_count+=1
        # This project's cmd_vel.angular.z is a nominal steering angle in rad.
        # It is intentionally not interpreted as a yaw-rate command.
        if not math.isfinite(m.linear.x) or not math.isfinite(m.angular.z):
            self.target=0.;self.brake=None;self.received=None
            self.steering_input=0.;self.steering_received=None;return
        self.target=m.linear.x*self.config['speed_to_erpm_gain']
        self.brake=None;self.received=self.time
        self.steering_input=m.angular.z;self.steering_received=self.time

    def command(self,m):
        self.callback_count+=1
        if not math.isfinite(m.data):
            self.target=0;self.brake=None;self.received=None;return
        self.target=m.data*self.config['erpm_sign'];self.brake=None;self.received=self.time

    def brake_command(self,m):
        self.callback_count+=1
        if not math.isfinite(m.data):
            self.target=0;self.brake=None;self.received=None;return
        self.brake=abs(m.data);self.received=self.time

    def spin(self,time):
        self.time=time
        # A 30 Hz viewport can receive 50+ Hz commands. Drain pending input
        # rather than leaving the actuator one growing queue behind the user.
        for _ in range(64):
            before=self.callback_count
            self.executor.spin_once(timeout_sec=0)
            if self.callback_count==before:break
        timeout=self.config['ros']['command_timeout_s']
        if self.steering_received is None or (timeout>0 and time-self.steering_received>timeout):
            self.steering_input=0.
        if self.received is None or (timeout>0 and time-self.received>timeout):
            return 0.0,None
        return self.target,self.brake

    def publish(self,time,erpm):
        sec=int(time);nsec=round((time-sec)*1e9)
        if nsec>=1_000_000_000:sec+=1;nsec-=1_000_000_000
        clock=self.clock_type();clock.clock.sec=sec;clock.clock.nanosec=nsec;self.clock.publish(clock)
        stopped=abs(erpm)<self.config['ros']['stop_threshold_erpm']
        if stopped and self.was_stopped and not self.config['ros']['publish_when_stopped']:return
        m=self.message_type();m.header.stamp.sec=sec;m.header.stamp.nanosec=nsec
        m.state.speed=erpm*self.config['erpm_sign']
        self.publisher.publish(m);self.was_stopped=stopped

    def close(self):
        self.executor.shutdown();self.node.destroy_node();self.rclpy.shutdown(context=self.context)
