define laser ranger (
	sensor(
	   range[ 0.0 5.6]
	   fov 240.0
	   samples 240
	   
 	)
	color "blue"
	size [0.06 0.06 0.06]
	
	block( points 4 
	   point[0] [0 0]
	   point[1] [0 1]
	   point[2] [1 1]
	   point[3] [1 0]
	   z [0 1]
	)
)


define crobot position(
    size [0.33 0.33 0.25]
    origin [0 0 0 0]
    gui_nose 1
    drive "diff"
    color "red"

    # This block approximates a circular shape of a Robot
    block( points 16
        point[0] [ 0.225 0.000 ]
        point[1] [ 0.208 0.086 ]
        point[2] [ 0.159 0.159 ]
        point[3] [ 0.086 0.208 ]
        point[4] [ 0.000 0.225 ]
        point[5] [ -0.086 0.208 ]
        point[6] [ -0.159 0.159 ]
        point[7] [ -0.208 0.086 ]
        point[8] [ -0.225 0.000 ]
        point[9] [ -0.208 -0.086 ]
        point[10] [ -0.159 -0.159 ]
        point[11] [ -0.086 -0.208 ]
        point[12] [ -0.000 -0.225 ]
        point[13] [ 0.086 -0.208 ]
        point[14] [ 0.159 -0.159 ]
        point[15] [ 0.208 -0.086 ]
        z [0 1]
    )
    

    # Report error-free position in world coordinates
    #localization "gps"
    #localization_origin [ 0 0 0 0 ]

    # Some more realistic localization error
    odom_error [ 0.01 0.01 0.0 0.1 ]
    localization "odom"
    laser(pose [0.23 0.0 -0.15 0.0])
    
    
)
