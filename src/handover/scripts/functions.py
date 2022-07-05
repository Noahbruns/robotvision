from tf.transformations import euler_from_quaternion

def euler_from_orientation(orientation):
    q = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]

    return euler_from_quaternion(q)