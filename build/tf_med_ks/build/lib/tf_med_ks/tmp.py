import numpy as np

tcp_points = [
    [0.35,  0.00, 0.25, 80, 0,   0],    
    [0.30,  0.20, 0.25, 18, 0,  30],   
    [0.30, -0.20, 0.25, 10, 0, -30],    
    [0.40,  0.00, 0.05, 90, 0,   0],    
    [0.40,  0.00, 0.45, 12, 0,   0],   
    [-0.20, 0.00, 0.25, 180, 0, 180]    
]

T = np.array([
    [ 0.99917906, -0.03103361, -0.02604083, -0.4301887 ],
    [ 0.03114451,  0.99950742,  0.00386378,  0.25203987],
    [ 0.02590809, -0.00467164,  0.99965341,  0.68007862],
    [ 0.0,         0.0,         0.0,         1.0       ]
])

tcp_points_transformed = []
for point in tcp_points:
    p = [point[0], point[1], point[2],1]
    #print(f"Point before transform: {p}")
    p_transformed = T.dot(p)
    #print(f"Point after transform: {p_transformed}")
    tcp_points_transformed.append([p_transformed[0], p_transformed[1], p_transformed[2],
                                     point[3], point[4], point[5]])
    print(f"Transformed TCP points so far:\n{tcp_points_transformed}")
