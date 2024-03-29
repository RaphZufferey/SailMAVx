uint64 timestamp			# time since system start (microseconds)

uint8 SWITCH_POS_NONE = 0		# switch is not mapped
uint8 SWITCH_POS_ON = 1		# switch activated (value = 1)
uint8 SWITCH_POS_MIDDLE = 2		# middle position (value = 0)
uint8 SWITCH_POS_OFF = 3		# switch not activated (value = -1)
uint8 MODE_SLOT_NONE = 0		# no mode slot assigned
uint8 MODE_SLOT_1 = 1			# mode slot 1 selected
uint8 MODE_SLOT_2 = 2			# mode slot 2 selected
uint8 MODE_SLOT_3 = 3			# mode slot 3 selected
uint8 MODE_SLOT_4 = 4			# mode slot 4 selected
uint8 MODE_SLOT_5 = 5			# mode slot 5 selected
uint8 MODE_SLOT_6 = 6			# mode slot 6 selected
uint8 MODE_SLOT_NUM = 6			# number of slots
uint8 SOURCE_RC = 1			# radio control
uint8 SOURCE_MAVLINK_0 = 2		# mavlink instance 0
uint8 SOURCE_MAVLINK_1 = 3		# mavlink instance 1
uint8 SOURCE_MAVLINK_2 = 4		# mavlink instance 2
uint8 SOURCE_MAVLINK_3 = 5		# mavlink instance 4

# Any of the channels may not be available and be set to NaN
# to indicate that it does not contain valid data.
# The variable names follow the definition of the
# MANUAL_CONTROL mavlink message.
# The default range is from -1 to 1 (mavlink message -1000 to 1000)
# The range for the z variable is defined from 0 to 1. (The z field of
# the MANUAL_CONTROL mavlink message is defined from -1000 to 1000)

float32 x			 # stick position in x direction -1..1
				 # in general corresponds to forward/back motion or pitch of vehicle,
				 # in general a positive value means forward or negative pitch and
				 # a negative value means backward or positive pitch
float32 y			 # stick position in y direction -1..1
				 # in general corresponds to right/left motion or roll of vehicle,
				 # in general a positive value means right or positive roll and
				 # a negative value means left or negative roll
float32 z			 # throttle stick position 0..1
				 # in general corresponds to up/down motion or thrust of vehicle,
				 # in general the value corresponds to the demanded throttle by the user,
				 # if the input is used for setting the setpoint of a vertical position
				 # controller any value > 0.5 means up and any value < 0.5 means down
float32 r			 # yaw stick/twist position, -1..1
				 # in general corresponds to the righthand rotation around the vertical
				 # (downwards) axis of the vehicle
float32 flaps			 # flap position
float32 aux1			 # default function: camera yaw / azimuth
float32 aux2			 # default function: camera pitch / tilt
float32 aux3			 # default function: camera trigger
float32 aux4			 # default function: camera roll
float32 aux5			 # default function: payload drop
float32 aux6

uint8 mode_switch		 # main mode 3 position switch (mandatory): _MANUAL_, ASSIST, AUTO
uint8 return_switch		 # return to launch 2 position switch (mandatory): _NORMAL_, RTL
uint8 rattitude_switch	 # rattitude control 2 position switch (optional): _MANUAL, RATTITUDE
uint8 posctl_switch		 # position control 2 position switch (optional): _ALTCTL_, POSCTL
uint8 loiter_switch		 # loiter 2 position switch (optional): _MISSION_, LOITER
uint8 acro_switch		 # acro 2 position switch (optional): _MANUAL_, ACRO
uint8 offboard_switch	 	 # offboard 2 position switch (optional): _NORMAL_, OFFBOARD
uint8 kill_switch		 # throttle kill: _NORMAL_, KILL
uint8 arm_switch		 # arm/disarm switch: _DISARMED_, ARMED
uint8 transition_switch	 # VTOL transition switch: _HOVER, FORWARD_FLIGHT
uint8 gear_switch	 # landing gear switch: _DOWN_, UP
uint8 mode_slot			 # the slot a specific model selector is in
uint8 data_source		 # where this input is coming from
uint8 stab_switch 		 # stabilize switch (only relevant for fixed wings, optional): _MANUAL, STABILIZED
uint8 man_switch		 # manual switch (only relevant for fixed wings, optional): _STABILIZED_, MANUAL

uint8 sail_switch 		# switch ON OFF for sailing controller