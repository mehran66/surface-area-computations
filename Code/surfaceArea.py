'''----------------------------------------------------------------------------------
File Name      : SurfaceArea.py
Author         : Mehran Ghandehari
Organization   : University of Colorado at Boulder
Created        : Nov. 10th, 2017
Python Version : 2.7

-- Description --
In this Module, surface area is calculated for the benchmark surface and differnt interpolation methods
----------------------------------------------------------------------------------'''
import numpy as np
#Import my modules
import neighbors
import bestFitting
import inverseDistanecWeighting
import findValue
import math



def distance3d(xp1,yp1,zp1,xp2,yp2,zp2):
    # Calculate 3D distance between two points
    d = ((xp1-xp2)**2 + (yp1-yp2)**2 + (zp1-zp2)**2)** 0.5
    return d

def areatriangle3d(x1,y1,z1,x2,y2,z2,x3,y3,z3):
    # Calculate area of a 3D triangle
    a = distance3d(x1,y1,z1,x2,y2,z2)
    b = distance3d(x2,y2,z2,x3,y3,z3)
    c = distance3d(x3,y3,z3,x1,y1,z1)

    # Heron method for calculating the area of a triangle when you know the lengths of all three sides.
    s = (a + b + c) / 2.0
    area = (s*(s-a)*(s-b)*(s-c)) ** 0.5

    return area

def trianglsSA(IntegralX1, IntegralY1, IntegralX2, IntegralY2, rasterBlock_x, rasterBlock_y, rasterBlock_elev, neibrs, order, xres, yres):

    SA = 0.0

    rasterBlock_x = rasterBlock_x - rasterBlock_x[2, 2]
    rasterBlock_y = rasterBlock_y - rasterBlock_y[2, 2]
    IntegralX1 = -(xres / 2.0)
    IntegralX2 = (xres / 2.0)
    IntegralY1 = -(yres / 2.0)
    IntegralY2 = (yres / 2.0)

    xMin = IntegralX1
    yMin = IntegralY1
    xMax = IntegralX2
    yMax = IntegralY2

    # Points on the boundary of central pixel that their elevation should be interpolated
    # 2    3    4
    # 1&9       5
    # 8    7    6
    pntX = [xMin, xMin, rasterBlock_x[2,2], xMax, xMax, xMax,rasterBlock_x[2,2], xMin, xMin]
    pntY = [rasterBlock_y[2,2], yMax, yMax, yMax, rasterBlock_y[2,2], yMin, yMin, yMin,rasterBlock_y[2,2]]


    for i in [1,2,3,4]: # Deviding the pixel into 4 squares, eahc of which contains two tiangles

        if order == 0: # IDW
            if i == 1:
                xCoor, yCoor, elev = neighbors.neibr(rasterBlock_x, rasterBlock_y, rasterBlock_elev, i, neibrs)
            elev1 = inverseDistanecWeighting.IDW(pntX[(i*2)-2], pntY[(i*2)-2], xCoor, yCoor, elev)
            elev2 = inverseDistanecWeighting.IDW(pntX[(i*2)-1], pntY[(i*2)-1], xCoor, yCoor, elev)
            elev3 = inverseDistanecWeighting.IDW(pntX[(i*2)], pntY[(i*2)], xCoor, yCoor, elev)

        elif order == 4: # BiQuadratic
            if i == 1:
                xCoor, yCoor, elev = neighbors.neibr(rasterBlock_x, rasterBlock_y, rasterBlock_elev, i, neibrs)
                m = bestFitting.polyfit2d(xCoor, yCoor, elev, order)
            elev1 = bestFitting.polyval2d(pntX[(i * 2) - 2], pntY[(i * 2) - 2], m)
            elev2 = bestFitting.polyval2d(pntX[(i * 2) - 1], pntY[(i * 2) - 1], m)
            elev3 = bestFitting.polyval2d(pntX[(i * 2)], pntY[(i * 2)], m)

        else:
            xCoor, yCoor, elev = neighbors.neibr(rasterBlock_x, rasterBlock_y, rasterBlock_elev, i, neibrs)
            m = bestFitting.polyfit2d(xCoor, yCoor, elev, order)
            elev1 = bestFitting.polyval2d(pntX[(i*2)-2], pntY[(i*2)-2], m)
            elev2 = bestFitting.polyval2d(pntX[(i*2)-1], pntY[(i*2)-1], m)
            elev3 = bestFitting.polyval2d(pntX[(i*2)], pntY[(i*2)], m)

        a1 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], pntX[(i*2)-2], pntY[(i*2)-2], elev1, pntX[(i*2)-1], pntY[(i*2)-1], elev2)
        a2 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], pntX[(i*2)-1], pntY[(i*2)-1], elev2, pntX[(i*2)], pntY[(i*2)], elev3)

        SA = SA + (a1 + a2)

    return SA


def area(mth, rasterBlock_elev, rasterBlock_x, rasterBlock_y, xres, yres, lidar):

    # calculating the x and y domain (square pixel) used in the surface area integrals
    IntegralX1 = rasterBlock_x[2, 2] - (xres / 2.0)
    IntegralX2 = rasterBlock_x[2, 2] + (xres / 2.0)
    IntegralY1 = rasterBlock_y[2, 2] - (yres / 2.0)
    IntegralY2 = rasterBlock_y[2, 2] + (yres / 2.0)

    if mth == 'Jenness':

        a1 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[2,1], rasterBlock_y[2,1], rasterBlock_elev[2,1], rasterBlock_x[1,1], rasterBlock_y[1,1], rasterBlock_elev[1,1])
        a2 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[1,1], rasterBlock_y[1,1], rasterBlock_elev[1,1], rasterBlock_x[1,2], rasterBlock_y[1,2], rasterBlock_elev[1,2])
        a3 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[1,2], rasterBlock_y[1,2], rasterBlock_elev[1,2], rasterBlock_x[1,3], rasterBlock_y[1,3], rasterBlock_elev[1,3])
        a4 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[1,3], rasterBlock_y[1,3], rasterBlock_elev[1,3], rasterBlock_x[2,3], rasterBlock_y[2,3], rasterBlock_elev[2,3])
        a5 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[2,3], rasterBlock_y[2,3], rasterBlock_elev[2,3], rasterBlock_x[3,3], rasterBlock_y[3,3], rasterBlock_elev[3,3])
        a6 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[3,3], rasterBlock_y[3,3], rasterBlock_elev[3,3], rasterBlock_x[3,2], rasterBlock_y[3,2], rasterBlock_elev[3,2])
        a7 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[3,2], rasterBlock_y[3,2], rasterBlock_elev[3,2], rasterBlock_x[3,1], rasterBlock_y[3,1], rasterBlock_elev[3,1])
        a8 = areatriangle3d(rasterBlock_x[2,2], rasterBlock_y[2,2], rasterBlock_elev[2,2], rasterBlock_x[3,1], rasterBlock_y[3,1], rasterBlock_elev[3,1], rasterBlock_x[2,1], rasterBlock_y[2,1], rasterBlock_elev[2,1])


        SA = (a1 + a2 + a3 + a4 + a5 + a6 +a7 + a8) / 4.0
        return SA

    elif mth == 'ben':
  
        elev1 = findValue.find(lidar, rasterBlock_x[2, 2], rasterBlock_y[2, 2])
        if elev1 < 0:
            return np.nan
        else:
            SA = 0.0

            # Extent of central pixel
            xMin = IntegralX1
            yMin = IntegralY1
            xMax = IntegralX2
            yMax = IntegralY2

            # points on the central pixel's boundary
            pntX = [xMin, xMin, rasterBlock_x[2, 2], xMax, xMax, xMax, rasterBlock_x[2, 2], xMin, xMin]
            pntY = [rasterBlock_y[2, 2], yMax, yMax, yMax, rasterBlock_y[2, 2], yMin, yMin, yMin, rasterBlock_y[2, 2]]
            elev = [findValue.find(lidar, pntX[i], pntY[i]) for i in range(len(pntX))]

            if (-1 in elev) or sum(n < 0 for n in elev)<0:
                return np.nan
            else:
                for i in [1, 2, 3, 4]:

                    a1 = areatriangle3d(rasterBlock_x[2, 2], rasterBlock_y[2, 2], elev1, pntX[(i * 2) - 2],
                                        pntY[(i * 2) - 2], elev[(i * 2) - 2], pntX[(i * 2) - 1], pntY[(i * 2) - 1], elev[(i * 2) - 1])
                    a2 = areatriangle3d(rasterBlock_x[2, 2], rasterBlock_y[2, 2], elev1, pntX[(i * 2) - 1],
                                        pntY[(i * 2) - 1], elev[(i * 2) - 1], pntX[(i * 2)], pntY[(i * 2)], elev[(i * 2)])

                    SA = SA + (a1 + a2)

        if SA < xres* yres:
            return np.nan
        else:
            return SA

    elif mth == 'slp':
        fx = ((2 * rasterBlock_elev[2, 3] + rasterBlock_elev[1, 3] + rasterBlock_elev[3, 3]) - (
                    2 * rasterBlock_elev[2, 1] + rasterBlock_elev[1, 1] + rasterBlock_elev[3, 1])) / (8.0 * xres)
        fy = ((2 * rasterBlock_elev[1, 2] + rasterBlock_elev[1, 1] + rasterBlock_elev[1, 3]) - (
                    2 * rasterBlock_elev[3, 2] + rasterBlock_elev[3, 1] + rasterBlock_elev[3, 3])) / (8.0 * xres)
        G = math.sqrt(1.0 + (fx ** 2.0) + (fy ** 2.0))
        SA = (xres ** 2.0) * G

        return SA

    elif mth == 'WA9':
        SA = trianglsSA(IntegralX1, IntegralY1, IntegralX2, IntegralY2, rasterBlock_x, rasterBlock_y, rasterBlock_elev, 9, 0, xres, yres)
        return SA

    elif mth == 'li':
        SA = trianglsSA(IntegralX1, IntegralY1, IntegralX2, IntegralY2, rasterBlock_x, rasterBlock_y, rasterBlock_elev, 3, 1, xres, yres)
        return SA

    elif mth == 'biLi4':
        SA = trianglsSA(IntegralX1, IntegralY1, IntegralX2, IntegralY2, rasterBlock_x, rasterBlock_y,
                        rasterBlock_elev, 4, 2, xres, yres)
        return SA


    elif mth == 'biQuad9':

        SA = trianglsSA(IntegralX1, IntegralY1, IntegralX2, IntegralY2, rasterBlock_x, rasterBlock_y,
                        rasterBlock_elev, 9, 4, xres, yres)

        return SA

    elif mth == 'biCub16':

        SA = trianglsSA(IntegralX1, IntegralY1, IntegralX2, IntegralY2, rasterBlock_x, rasterBlock_y,
                        rasterBlock_elev, 16, 5, xres, yres)

        return SA










