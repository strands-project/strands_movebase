#! /usr/bin/env python

import sys
import yaml
import rospy
import dynamic_reconfigure.client


if __name__ == '__main__':
    rospy.init_node('param_loader')
    
    filtered_argv=rospy.myargv(argv=sys.argv)
    
    if len(filtered_argv)<2:
        rospy.logwarn("No config yaml file provided.")
    elif len(filtered_argv)>2:
        rospy.logwarn("Too many arguments.")
    else:
        file_name=filtered_argv[1]
        
    
    with open(file_name, 'r') as stream:
        yaml_config=yaml.load(stream)
        print yaml_config
        for key, val in yaml_config.iteritems():
            print key
            print val
            client = dynamic_reconfigure.client.Client("/move_base/" + key)
            client.update_configuration(val)
        print "UPDATED"
   
