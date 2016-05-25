#!/usr/bin/env python

import rospy
from cmd_id_generator.srv import *

CMD_ID = 2147483647 # CMD_ID init by max value of int32 
def generate_cmd_id(req):
	global CMD_ID
	print "new id %s"%CMD_ID
	CMD_ID -= 1
	return CmdIdResponse(CMD_ID + 1);

def generate_cmd_id_server():
	rospy.init_node('gen_cmd_id_node')
	rospy.Service('generate_cmd_id', CmdId, generate_cmd_id)
	print "Ready to generate command id"
	rospy.spin()

if __name__ == "__main__":
	generate_cmd_id_server()