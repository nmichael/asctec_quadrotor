PKG= asctec
MEX_MSGS= Command Status
ROS_MSGS= Command Status

MEX_FILES := $(foreach msg, $(MEX_MSGS), $(PKG)_$(msg))
ROS_FILES := $(foreach msg, $(ROS_MSGS), $(PKG)_$(msg)_publisher)
ROS_FILES += $(foreach msg, $(ROS_MSGS), $(PKG)_$(msg)_subscriber)

include $(shell rospack find ipc_rosgraph_msgs)/ipc_rosgraph_msgs.mk
include $(shell rospack find ipc_geometry_msgs)/ipc_geometry_msgs.mk
include $(shell rospack find ipc_bridge_local)/ipc_bridge_local.mk
