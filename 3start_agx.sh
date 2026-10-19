#!/bin/bash

source pico_tele_agx/install/setup.bash
ros2 launch agx_arm_ctrl start_dual_agx_arm.launch.py left_can_port:=can_left right_can_port:=can_right arm_type:=piper left_effector_type:=agx_gripper right_effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
