import rospy

from geometry_msgs.msg import Twist
from keyboard.msg import Key

vel = Twist()

def callback(command):
	if command.code == 97:
		print 'yes w'
		vel.linear.x = 5
	elif command.code =='s':
		vel.linear.x = 0
	elif command.code == 'a':
		vel.linear.y = 5
	elif command.code == 'd':
		vel.linear.y = 0

	print 'vel:',vel

def main():
	rospy.init_node('Polaris_control_node',anonymous=True)
	rospy.Subscriber('/keyboard/keydown',Key,callback)
	vel_pub = rospy.Publisher('/polaris/cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		vel_pub.publish(vel)
		rate.sleep()

if __name__ == '__main__':
	main()