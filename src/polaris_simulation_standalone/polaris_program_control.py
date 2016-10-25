import rospy

from geometry_msgs.msg import Twist

vel = Twist()

M_PI = 3.1415926

def main():
	rospy.init_node('Polaris_program_control_node',anonymous=True)
	vel_pub = rospy.Publisher('/polaris/cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(10)
	counter = 0
	while not rospy.is_shutdown():
		counter += 1
		if counter <= 448:
			
			vel.linear.x = 1
			vel.angular.z = 0
		elif counter > 448 and counter <= 453:
			print "turn"
			vel.linear.x = 1
			vel.angular.z = 30*M_PI/180.0
		elif counter > 453:
			vel.linear.x = 1
			vel.angular.z = 0

		vel_pub.publish(vel)
		rate.sleep()

if __name__ == '__main__':
	main()