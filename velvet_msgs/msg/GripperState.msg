#this message defines the state of a velvet2 gripper
#encoders
EncoderState oc #open-close
EncoderState blf #belt left-front
EncoderState blb #belt left-back
EncoderState brf #belt right-front
EncoderState brb #belt right-back
EncoderState pl #phalange left
EncoderState pr #phalange right
#current sensors
CurrentState c_oc #open-close
CurrentState c_blf #belt left-front
CurrentState c_blb #belt left-back
CurrentState c_brf #belt right-front
CurrentState c_brb #belt right-back
#control mode
char oc_mode # C = current, P = position, V = velocity
char b_mode  # same for belts
