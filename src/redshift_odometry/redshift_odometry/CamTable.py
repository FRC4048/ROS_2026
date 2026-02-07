class CamTable():
    
    # transformation of cam relative to the robot. x, y, z is translation, roll, pitch, yaw is rotation.
    #
    # NOTE: The robot is in FLU frame. We need to turn the camera to RDF (this is how apriltag_ros publishes transformations).
    # to do that we apply a ZYX rotation of x=-90, y=0, z=-90
    #
    #       the rotation of camera relative to the robot has to be applied on top of that
    #       https://www.andre-gaschler.com/rotationconverter/ is helpfull....
    # 
    
    # this table has the quat transform of the camera AFTER rotating it to RDF...
    # if there is no additional rotation, set the quaternion to qx,qy,qz,qw of 0,0,0,1
    cam_table = [
      {"camid": "cam1" , "x": 0.3015, "y": -0.251, "z": 0.18, 
                         "qx": 0.0434534, "qy": -0.0870728, "qz": 0.0038017, "qw": 0.9952465},     # new reef camera 10 left, 5 up
      # {"camid": "cam2" , "x": -0.07, "y": 0.2446, "z": 0.988, 
      #                    "qx": 0.0301537, "qy": 0.9698463, "qz": -0.1710101, "qw": 0.1710101},     #elevator cam: 200 left, 20 up    
      {"camid": "cam2" , "x": 0.3496, "y": 0.003, "z": 0.3, 
                         "qx": 0, "qy": 0.1391731, "qz": 0, "qw": 0.9902681},     #under climber cam: 16 right
    ]
    
    
    def compound_quat(entry):
       # following quaternion is the adjustment of the camera from FLU To RDF
       x2, y2, z2, w2 = -0.5, 0.5, -0.5, 0.5
                     
       x1 = entry["qx"]
       y1 = entry["qy"]
       z1 = entry["qz"]       
       w1 = entry["qw"]
       
       w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
       x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
       y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
       z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
       
       return w, x, y, z
       
