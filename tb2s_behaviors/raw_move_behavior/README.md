# raw_move_behavior

# This behavior will do a "Raw" move, meaning, it does not use sensors to determine path or safety!  It just moves!

# To test, send distance and speed:
  rostopic pub -1 /behavior/cmd behavior_common/CommandState 
    "RAW_MOVE" “’1.0’” "'0.5'"  <-- double quotes around numbers!
