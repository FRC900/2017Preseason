#s is a message that holds data to describe the state of a set of torque
#controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All
# the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint
# state. 
# The goal is to make each of the fields optional. When e.g. your joints have
# no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
#RG: consider adding rumble as part of the msg
Header header

string[] name
float64[] leftStickX
float64[] leftStickY
float64[] rightStickX
float64[] rightStickY
float64[] leftTrigger
float64[] rightTrigger
bool[] buttonX
bool[] buttonXPress
bool[] buttonXRelease
bool[] buttonY
bool[] buttonYPress
bool[] buttonYRelease
bool[] buttonA
bool[] buttonAPress
bool[] buttonARelease
bool[] buttonB
bool[] buttonBPress
bool[] buttonBRelease
bool[] bumperLeft
bool[] bumperLeftPress
bool[] bumperLeftRelease
bool[] bumperRight
bool[] bumperRightPress
bool[] bumperRightRelease
bool[] buttonBack
bool[] buttonBackPress
bool[] buttonBackRelease
bool[] buttonStart
bool[] buttonStartPress
bool[] buttonStartRelease
bool[] stickLeft
bool[] stickLeftPress
bool[] stickLeftRelease
bool[] stickRight
bool[] stickRightPress
bool[] stickRightRelease
bool[] dirLeft
bool[] dirLeftPress
bool[] dirLeftRelease
bool[] dirUp
bool[] dirUpPress
bool[] dirUpRelease
bool[] dirRight
bool[] dirRightPress
bool[] dirRightRelease
bool[] dirDown
bool[] dirDownPress
bool[] dirDownRelease
