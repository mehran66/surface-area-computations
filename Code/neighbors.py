import numpy as np

'''Zone
Each pixels is devided into 4 parts as coded in the following
--------------------
|        |          |
|  (1)   |  (2)     |
|        |          |
|________|__________|        
|        |          |
|   (4)  |   (3)    |
|        |          |
|________|__________|        

'''


def neibr(rasterBlock_x, rasterBlock_y, rasterBlock_elev, zone, m):
    '''
    -This function create the neighborhod matrices
    -rasterBlock is the 5 by 5 matrix
    '''

    if m == 1:
        xCoor = rasterBlock_x[2,2]
        yCoor = rasterBlock_y[2,2]
        elev = rasterBlock_elev[2,2]

    elif m == 3:

        if zone == 1:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,1], rasterBlock_x[1,2]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,1], rasterBlock_y[1,2]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,1], rasterBlock_elev[1,2]])
        elif zone == 4:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,1], rasterBlock_x[3,2]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,1], rasterBlock_y[3,2]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,1], rasterBlock_elev[3,2]])
        if zone == 2:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x[1,2], rasterBlock_x[2,3]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y[1,2], rasterBlock_y[2,3]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev[1,2], rasterBlock_elev[2,3]])
        elif zone == 3:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,3], rasterBlock_x[3,2]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,3], rasterBlock_y[3,2]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,3], rasterBlock_elev[3,2]])

    elif m ==4:
        if zone == 1:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,1], rasterBlock_x[1,1], rasterBlock_x[1,2]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,1], rasterBlock_y[1,1], rasterBlock_y[1,2]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,1], rasterBlock_elev[1,1], rasterBlock_elev[1,2]])
        elif zone == 4:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,1], rasterBlock_x[3,1], rasterBlock_x[3,2]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,1], rasterBlock_y[3,1], rasterBlock_y[3,2]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,1], rasterBlock_elev[3,1], rasterBlock_elev[3,2]])
        if zone == 2:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [1,3], rasterBlock_x[1,2], rasterBlock_x[2,3]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [1,3], rasterBlock_y[1,2], rasterBlock_y[2,3]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [1,3], rasterBlock_elev[1,2], rasterBlock_elev[2,3]])
        elif zone == 3:
            xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,3], rasterBlock_x[3,2], rasterBlock_x[3,3]])
            yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,3], rasterBlock_y[3,2], rasterBlock_y[3,3]])
            elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,3], rasterBlock_elev[3,2], rasterBlock_elev[3,3]])


    elif m ==5:
        xCoor = np.array([rasterBlock_x[2,2], rasterBlock_x [2,1], rasterBlock_x[1,2], rasterBlock_x[2,3], rasterBlock_x[3,2]])
        yCoor = np.array([rasterBlock_y[2,2], rasterBlock_y [2,1], rasterBlock_y[1,2], rasterBlock_y[2,3], rasterBlock_y[3,2]])
        elev = np.array([rasterBlock_elev[2,2], rasterBlock_elev [2,1], rasterBlock_elev[1,2], rasterBlock_elev[2,3], rasterBlock_elev[3,2]])

    elif m ==8:
        xCoor = np.array([rasterBlock_x [2,1], rasterBlock_x[1,2], rasterBlock_x[2,3], rasterBlock_x[3,2], rasterBlock_x[1,1],
                          rasterBlock_x[1,3], rasterBlock_x[3,1], rasterBlock_x[3,3]])
        yCoor = np.array([rasterBlock_y [2,1], rasterBlock_y[1,2], rasterBlock_y[2,3], rasterBlock_y[3,2], rasterBlock_y[1,1],
                          rasterBlock_y[1,3], rasterBlock_y[3,1], rasterBlock_y[3,3]])
        elev = np.array([rasterBlock_elev [2,1], rasterBlock_elev[1,2], rasterBlock_elev[2,3], rasterBlock_elev[3,2], rasterBlock_elev[1,1],
                          rasterBlock_elev[1,3], rasterBlock_elev[3,1], rasterBlock_elev[3,3]])

    elif m ==9:
        xCoor = np.array([rasterBlock_x [2,2], rasterBlock_x[1,2], rasterBlock_x[2,3], rasterBlock_x[3,2], rasterBlock_x[1,1],
                          rasterBlock_x[1,3], rasterBlock_x[2,1], rasterBlock_x[3,1], rasterBlock_x[3,3]])
        yCoor = np.array([rasterBlock_y [2,2], rasterBlock_y[1,2], rasterBlock_y[2,3], rasterBlock_y[3,2], rasterBlock_y[1,1],
                          rasterBlock_y[1,3], rasterBlock_y[2,1], rasterBlock_y[3,1], rasterBlock_y[3,3]])
        elev = np.array([rasterBlock_elev [2,2], rasterBlock_elev[1,2], rasterBlock_elev[2,3], rasterBlock_elev[3,2], rasterBlock_elev[1,1],
                          rasterBlock_elev[1,3], rasterBlock_elev[2,1], rasterBlock_elev[3,1], rasterBlock_elev[3,3]])

    elif m ==16:
        if zone ==1:
            xCoor = np.array([rasterBlock_x [2,2], rasterBlock_x[0,1], rasterBlock_x[0,2], rasterBlock_x[0,3],
                             rasterBlock_x [1,0], rasterBlock_x[1,1], rasterBlock_x[1,2], rasterBlock_x[1,3],
                             rasterBlock_x [2,0], rasterBlock_x[2,1], rasterBlock_x[0,0], rasterBlock_x[2,3],
                             rasterBlock_x [3,0], rasterBlock_x[3,1], rasterBlock_x[3,2], rasterBlock_x[3,3]])
            yCoor = np.array([rasterBlock_y [2,2], rasterBlock_y[0,1], rasterBlock_y[0,2], rasterBlock_y[0,3],
                             rasterBlock_y [1,0], rasterBlock_y[1,1], rasterBlock_y[1,2], rasterBlock_y[1,3],
                             rasterBlock_y [2,0], rasterBlock_y[2,1], rasterBlock_y[0,0], rasterBlock_y[2,3],
                             rasterBlock_y [3,0], rasterBlock_y[3,1], rasterBlock_y[3,2], rasterBlock_y[3,3]])
            elev = np.array([rasterBlock_elev [2,2], rasterBlock_elev[0,1], rasterBlock_elev[0,2], rasterBlock_elev[0,3],
                             rasterBlock_elev [1,0], rasterBlock_elev[1,1], rasterBlock_elev[1,2], rasterBlock_elev[1,3],
                             rasterBlock_elev [2,0], rasterBlock_elev[2,1], rasterBlock_elev[0,0], rasterBlock_elev[2,3],
                             rasterBlock_elev [3,0], rasterBlock_elev[3,1], rasterBlock_elev[3,2], rasterBlock_elev[3,3]])
        elif zone ==4:
            xCoor = np.array([rasterBlock_x [2,2], rasterBlock_x[4,1], rasterBlock_x[4,2], rasterBlock_x[4,3],
                             rasterBlock_x [1,0], rasterBlock_x[1,1], rasterBlock_x[1,2], rasterBlock_x[1,3],
                             rasterBlock_x [2,0], rasterBlock_x[2,1], rasterBlock_x[4,0], rasterBlock_x[2,3],
                             rasterBlock_x [3,0], rasterBlock_x[3,1], rasterBlock_x[3,2], rasterBlock_x[3,3]])
            yCoor = np.array([rasterBlock_y [2,2], rasterBlock_y[4,1], rasterBlock_y[4,2], rasterBlock_y[4,3],
                             rasterBlock_y [1,0], rasterBlock_y[1,1], rasterBlock_y[1,2], rasterBlock_y[1,3],
                             rasterBlock_y [2,0], rasterBlock_y[2,1], rasterBlock_y[4,0], rasterBlock_y[2,3],
                             rasterBlock_y [3,0], rasterBlock_y[3,1], rasterBlock_y[3,2], rasterBlock_y[3,3]])
            elev = np.array([rasterBlock_elev [2,2], rasterBlock_elev[4,1], rasterBlock_elev[4,2], rasterBlock_elev[4,3],
                             rasterBlock_elev [1,0], rasterBlock_elev[1,1], rasterBlock_elev[1,2], rasterBlock_elev[1,3],
                             rasterBlock_elev [2,0], rasterBlock_elev[2,1], rasterBlock_elev[4,0], rasterBlock_elev[2,3],
                             rasterBlock_elev [3,0], rasterBlock_elev[3,1], rasterBlock_elev[3,2], rasterBlock_elev[3,3]])
        if zone ==2:
            xCoor = np.array([rasterBlock_x [2,2], rasterBlock_x[0,1], rasterBlock_x[0,2], rasterBlock_x[0,3],
                             rasterBlock_x [1,4], rasterBlock_x[1,1], rasterBlock_x[1,2], rasterBlock_x[1,3],
                             rasterBlock_x [2,4], rasterBlock_x[2,1], rasterBlock_x[0,4], rasterBlock_x[2,3],
                             rasterBlock_x [3,4], rasterBlock_x[3,1], rasterBlock_x[3,2], rasterBlock_x[3,3]])
            yCoor = np.array([rasterBlock_y [2,2], rasterBlock_y[0,1], rasterBlock_y[0,2], rasterBlock_y[0,3],
                             rasterBlock_y [1,4], rasterBlock_y[1,1], rasterBlock_y[1,2], rasterBlock_y[1,3],
                             rasterBlock_y [2,4], rasterBlock_y[2,1], rasterBlock_y[0,4], rasterBlock_y[2,3],
                             rasterBlock_y [3,4], rasterBlock_y[3,1], rasterBlock_y[3,2], rasterBlock_y[3,3]])
            elev = np.array([rasterBlock_elev [2,2], rasterBlock_elev[0,1], rasterBlock_elev[0,2], rasterBlock_elev[0,3],
                             rasterBlock_elev [1,4], rasterBlock_elev[1,1], rasterBlock_elev[1,2], rasterBlock_elev[1,3],
                             rasterBlock_elev [2,4], rasterBlock_elev[2,1], rasterBlock_elev[0,4], rasterBlock_elev[2,3],
                             rasterBlock_elev [3,4], rasterBlock_elev[3,1], rasterBlock_elev[3,2], rasterBlock_elev[3,3]])
        elif zone ==3:
            xCoor = np.array([rasterBlock_x [2,2], rasterBlock_x[4,1], rasterBlock_x[4,2], rasterBlock_x[4,3],
                             rasterBlock_x [1,4], rasterBlock_x[1,1], rasterBlock_x[1,2], rasterBlock_x[1,3],
                             rasterBlock_x [2,4], rasterBlock_x[2,1], rasterBlock_x[4,4], rasterBlock_x[2,3],
                             rasterBlock_x [3,4], rasterBlock_x[3,1], rasterBlock_x[3,2], rasterBlock_x[3,3]])
            yCoor = np.array([rasterBlock_y [2,2], rasterBlock_y[4,1], rasterBlock_y[4,2], rasterBlock_y[4,3],
                             rasterBlock_y [1,4], rasterBlock_y[1,1], rasterBlock_y[1,2], rasterBlock_y[1,3],
                             rasterBlock_y [2,4], rasterBlock_y[2,1], rasterBlock_y[4,4], rasterBlock_y[2,3],
                             rasterBlock_y [3,4], rasterBlock_y[3,1], rasterBlock_y[3,2], rasterBlock_y[3,3]])
            elev = np.array([rasterBlock_elev [2,2], rasterBlock_elev[4,1], rasterBlock_elev[4,2], rasterBlock_elev[4,3],
                             rasterBlock_elev [1,4], rasterBlock_elev[1,1], rasterBlock_elev[1,2], rasterBlock_elev[1,3],
                             rasterBlock_elev [2,4], rasterBlock_elev[2,1], rasterBlock_elev[4,4], rasterBlock_elev[2,3],
                             rasterBlock_elev [3,4], rasterBlock_elev[3,1], rasterBlock_elev[3,2], rasterBlock_elev[3,3]])

    elif m ==17:
        xCoor = np.array([rasterBlock_x [2,2], rasterBlock_x[1,2], rasterBlock_x[2,3], rasterBlock_x[3,2], rasterBlock_x[1,1],
                      rasterBlock_x[1,3], rasterBlock_x[3,1], rasterBlock_x[3,3], rasterBlock_x[0,0], rasterBlock_x[0,2], rasterBlock_x[0,4],
                          rasterBlock_x[2,0], rasterBlock_x[2,1], rasterBlock_x[2,4], rasterBlock_x[4,0], rasterBlock_x[4,2], rasterBlock_x[4,4]])
        yCoor = np.array([rasterBlock_y [2,2], rasterBlock_y[1,2], rasterBlock_y[2,3], rasterBlock_y[3,2], rasterBlock_y[1,1],
                      rasterBlock_y[1,3], rasterBlock_y[3,1], rasterBlock_y[3,3], rasterBlock_y[0,0], rasterBlock_y[0,2], rasterBlock_y[0,4],
                          rasterBlock_y[2,0], rasterBlock_y[2,1], rasterBlock_y[2,4], rasterBlock_y[4,0], rasterBlock_y[4,2], rasterBlock_y[4,4]])
        elev = np.array([rasterBlock_elev [2,2], rasterBlock_elev[1,2], rasterBlock_elev[2,3], rasterBlock_elev[3,2], rasterBlock_elev[1,1],
                      rasterBlock_elev[1,3], rasterBlock_elev[3,1], rasterBlock_elev[3,3], rasterBlock_elev[0,0], rasterBlock_elev[0,2], rasterBlock_elev[0,4],
                          rasterBlock_elev[2,0], rasterBlock_elev[2,1], rasterBlock_elev[2,4], rasterBlock_elev[4,0], rasterBlock_elev[4,2], rasterBlock_elev[4,4]])
    elif m ==25:
        xCoor = rasterBlock_x.flatten()
        yCoor = rasterBlock_y.flatten()
        elev = rasterBlock_elev.flatten()

    return xCoor, yCoor, elev