# -*- coding: utf-8 -*-
'''----------------------------------------------------------------------------------
File Name      : surfacearea.py
Author         : Mehran Ghandehari
Organization   : University of Colorado at Boulder
Created        : Feb. 02th, 2018
Modified       : Mar. 05th, 2021
Python Version : 3.7
License        : MIT License

-- Description --
Surface-adjusted area is calculated in this code using different interpolation methods and different contiguity configurations across different resolutions.

Input and output:
Input folder (./data") should include all of the DEM files (tif files)
All of the results (surface area rasters) are stored in the ./output folder
Number of CPUs to be used in parallel processing

References:
Ghandehari M, P. Buttenfield B. 2018. Slope-Adjusted Surface Area Computations in Digital Terrain. PeerJ Preprints 6:e27068v1 https://doi.org/10.7287/peerj.preprints.27068v1
----------------------------------------------------------------------------------'''
# Import modules
try:
    import multiprocessing
    import rasterio
    import os
    import numpy as np
    from numpy.linalg import inv
    from time import time
    from osgeo import gdal
    import math
    import argparse
    import numbers
    from collections import namedtuple

except ImportError as e:
    print('Could not import necessary libraries')
    print(e)
    raise ImportError

class Raster:

    def __init__(self, raster):
        self._raster = raster
        self._geo = gdal.Open(raster)  # open the geotif
        self._minX, self._xres, self._xskew, self._maxY, self._yskew, self._yres = self._geo.GetGeoTransform() # get raster transformation files
        self._maxX = self._minX + (self._geo.RasterXSize * self._xres) # calculate maximum x of the raster object
        self._minY = self._maxY + (self._geo.RasterYSize * self._yres) # calculate minimum y of the raster object
        self._sourceProj = self._geo.GetProjectionRef() # Setup the source projection - you can also import from epsg, proj4...
        self._ndval = self._geo.GetRasterBand(1).GetNoDataValue() # get nodata value

    @property
    def xres(self):
        return self._xres

    @property
    def yres(self):
        return self._yres

    @property
    def sourceProj(self):
        return self._sourceProj

    @property
    def ndval(self):
        return self._ndval

    def raster2Numpy(self):
        '''
        Convert a Geotif rater to a Numpy array

        return: numpy array X, Y, and elevation
        '''
        elev = self._geo.ReadAsArray() # read the the raster object as a numpy array
        # generate  coordinate (X and Y) matrices
        X = np.arange(self._minX + (self._xres / 2.0), self._maxX, self._xres)
        Y = np.arange(self._minY + abs(self._yres / 2.0), self._maxY, abs(self._yres))
        X, Y = np.meshgrid(X, Y)
        Y = Y[::-1]
        return X, Y, elev

    def numpy2Raster(self, inputArray, outputRaster):
        '''
        Covert a numpy array to a raster geotif file

        :inputArray: the input numpy array that is going to convert to a geotiff
        :outputRaster: where the geotiff file saves
        '''
        nrows, ncols = np.shape(inputArray)  # Array shape
        geotransform = (self._minX, self._xres, self._xskew, self._maxY, self._yskew, self._yres) # set transformation parameters
        output_raster = gdal.GetDriverByName('GTiff').Create(outputRaster + '.tif', ncols, nrows, 1 , gdal.GDT_Float32)  # Open the output file
        output_raster.SetGeoTransform(geotransform)  # Specify its geotransformation
        output_raster.SetProjection(self._sourceProj)   # Exports the coordinate system to the file
        output_raster.GetRasterBand(1).SetNoDataValue(self._ndval)  # set no data value
        output_raster.GetRasterBand(1).WriteArray(inputArray)   # Writes the array to the raster

    def __repr__(self):
        return (f"Raster(Min X='{self.minX}', "
                f"Min Y={self.minY}, "
                f"Max X={self.maxX}, "
                f"Max Y={self.maxY}, "
                f"No Data Value={self.ndval}, "
                f"Projection={self.sourceProj})")

class Interpolation:

    def __init__(self, rasterBlock_x, rasterBlock_y, rasterBlock_elev, zone, neibrs):
        self.rasterBlock_x = rasterBlock_x
        self.rasterBlock_y = rasterBlock_y
        self.rasterBlock_elev = rasterBlock_elev
        self.zone = zone
        self.neibrs = neibrs
        self.xCoords, self.yCoords, self.elevs = self.findNeibr()

    def polyfit2d(self, order):
        '''
        Calcluate the coefficients of a polynomial

        :order: identify the polynomial type
        '''
        if order == 1:  # Linear
            G = np.vstack([np.ones(len(self.xCoords)), self.xCoords, self.yCoords]).T
        if order == 2:  # BiLinear
            G = np.vstack([np.ones(len(self.xCoords)), self.xCoords, self.yCoords, self.xCoords * self.yCoords]).T
        if order == 3:  # BiQuadratic6
            G = np.vstack([np.ones(len(self.xCoords)), self.xCoords, self.yCoords, self.xCoords * self.yCoords, self.xCoords ** 2,
                           self.yCoords ** 2]).T
        if order == 4:  # BiQuadratic9
            G = np.vstack([np.ones(len(self.xCoords)), self.xCoords, self.yCoords, self.xCoords * self.yCoords, self.xCoords ** 2, self.yCoords ** 2,
                           self.xCoords ** 2 * self.yCoords ** 2, self.xCoords ** 2 * self.yCoords, self.yCoords ** 2 * self.xCoords]).T
        if order == 5:  # BiCubic
            G = np.vstack(
                [np.ones(len(self.xCoords)), self.xCoords, self.yCoords, self.xCoords * self.yCoords, self.xCoords ** 2, self.yCoords ** 2,
                 self.xCoords ** 3, self.yCoords ** 3, self.xCoords ** 2 * self.yCoords, self.yCoords ** 2 * self.xCoords,
                 self.xCoords ** 3 * self.yCoords, self.yCoords ** 3 * self.xCoords, self.xCoords ** 2 * self.yCoords ** 2,
                 self.yCoords ** 3 * self.xCoords ** 3, self.yCoords ** 3 * self.xCoords ** 2, self.yCoords ** 2 * self.xCoords ** 3]).T

        try:
            m = np.matmul(inv(G), self.elevs)
        except np.linalg.LinAlgError:
            print("Singular matrix")
            m = np.zeros(len(self.elevs))
            pass

        return m

    def polyval2d(self, x, y, m):
        '''
        Interpolated the z value of a point based on polynomial surfaces

        :x: x coordinate
        :y: y coordinate
        :m: polynomial coefficient calculated in the polyfit2d method
        '''
        if len(m) == 3:
            z = np.dot(np.array([1, x, y]), m)
        if len(m) == 4:
            z = np.dot(np.array([1, x, y, x * y]), m)
        if len(m) == 6:
            z = np.dot(np.array([1, x, y, x * y, x ** 2, y ** 2]), m)
        if len(m) == 9:
            z = np.dot(np.array([1, x, y, x * y, x ** 2, y ** 2, x ** 2 * y ** 2, x ** 2 * y, y ** 2 * x]), m)
        if len(m) == 16:
            z = np.dot(np.array(
                [1, x, y, x * y, x ** 2, y ** 2, x ** 3, y ** 3, x ** 2 * y, y ** 2 * x, x ** 3 * y, y ** 3 * x,
                 x ** 2 * y ** 2, y ** 3 * x ** 3, y ** 3 * x ** 2, y ** 2 * x ** 3]), m)
        return z

    def idw(self, x, y, order=2):
        '''
        Interpolated the z value of a point based on Weighted Average interpolation method

        :x: x coordinate
        :y: y coordinate
        :m: the power of distance in the inverse distance that is used as a weight in IDW
        '''

        d = np.zeros(len(self.xCoords))
        w = np.zeros(len(self.xCoords))
        for i in range(len(self.xCoords)):
            d[i] = math.sqrt((x - self.xCoords[i]) ** 2 + (y - self.yCoords[i]) ** 2)
            w[i] = 1.0 / (d[i] ** order)
        z = (np.sum(self.elevs * w)) / float(np.sum(w))
        return z

    def findNeibr(self):
        '''
        This function use the rasterBlock that is a 5 by 5 matrix
        and extract a sub-windows of the original matrix based on
        the zone and number of cells

        :zone: Each pixels is devided into 4 parts as coded in the following
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


        if self.neibrs == 3:
            if self.zone == 1:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[2, 1], self.rasterBlock_x[1, 2]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[2, 1], self.rasterBlock_y[1, 2]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[1, 2]])
            elif self.zone == 4:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[2, 1], self.rasterBlock_x[3, 2]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[2, 1], self.rasterBlock_y[3, 2]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[3, 2]])
            if self.zone == 2:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[1, 2], self.rasterBlock_x[2, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[1, 2], self.rasterBlock_y[2, 3]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[2, 3]])
            elif self.zone == 3:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[2, 3], self.rasterBlock_x[3, 2]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[2, 3], self.rasterBlock_y[3, 2]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[2, 3], self.rasterBlock_elev[3, 2]])

        elif self.neibrs == 4:
            if self.zone == 1:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[2, 1], self.rasterBlock_x[1, 1], self.rasterBlock_x[1, 2]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[2, 1], self.rasterBlock_y[1, 1], self.rasterBlock_y[1, 2]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[1, 1], self.rasterBlock_elev[1, 2]])
            elif self.zone == 4:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[2, 1], self.rasterBlock_x[3, 1], self.rasterBlock_x[3, 2]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[2, 1], self.rasterBlock_y[3, 1], self.rasterBlock_y[3, 2]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[3, 1], self.rasterBlock_elev[3, 2]])
            if self.zone == 2:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[1, 3], self.rasterBlock_x[1, 2], self.rasterBlock_x[2, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[1, 3], self.rasterBlock_y[1, 2], self.rasterBlock_y[2, 3]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[1, 3], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[2, 3]])
            elif self.zone == 3:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[2, 3], self.rasterBlock_x[3, 2], self.rasterBlock_x[3, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[2, 3], self.rasterBlock_y[3, 2], self.rasterBlock_y[3, 3]])
                elev = np.array([self.rasterBlock_elev[2, 2], self.rasterBlock_elev[2, 3], self.rasterBlock_elev[3, 2], self.rasterBlock_elev[3, 3]])

        elif self.neibrs == 9:
            xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[1, 2], self.rasterBlock_x[2, 3], self.rasterBlock_x[3, 2],
                              self.rasterBlock_x[1, 1],
                              self.rasterBlock_x[1, 3], self.rasterBlock_x[2, 1], self.rasterBlock_x[3, 1], self.rasterBlock_x[3, 3]])
            yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[1, 2], self.rasterBlock_y[2, 3], self.rasterBlock_y[3, 2],
                              self.rasterBlock_y[1, 1],
                              self.rasterBlock_y[1, 3], self.rasterBlock_y[2, 1], self.rasterBlock_y[3, 1], self.rasterBlock_y[3, 3]])
            elev = np.array(
                [self.rasterBlock_elev[2, 2], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[2, 3], self.rasterBlock_elev[3, 2],
                 self.rasterBlock_elev[1, 1],
                 self.rasterBlock_elev[1, 3], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[3, 1], self.rasterBlock_elev[3, 3]])

        elif self.neibrs == 16:
            if self.zone == 1:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[0, 1], self.rasterBlock_x[0, 2], self.rasterBlock_x[0, 3],
                                  self.rasterBlock_x[1, 0], self.rasterBlock_x[1, 1], self.rasterBlock_x[1, 2], self.rasterBlock_x[1, 3],
                                  self.rasterBlock_x[2, 0], self.rasterBlock_x[2, 1], self.rasterBlock_x[0, 0], self.rasterBlock_x[2, 3],
                                  self.rasterBlock_x[3, 0], self.rasterBlock_x[3, 1], self.rasterBlock_x[3, 2], self.rasterBlock_x[3, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[0, 1], self.rasterBlock_y[0, 2], self.rasterBlock_y[0, 3],
                                  self.rasterBlock_y[1, 0], self.rasterBlock_y[1, 1], self.rasterBlock_y[1, 2], self.rasterBlock_y[1, 3],
                                  self.rasterBlock_y[2, 0], self.rasterBlock_y[2, 1], self.rasterBlock_y[0, 0], self.rasterBlock_y[2, 3],
                                  self.rasterBlock_y[3, 0], self.rasterBlock_y[3, 1], self.rasterBlock_y[3, 2], self.rasterBlock_y[3, 3]])
                elev = np.array(
                    [self.rasterBlock_elev[2, 2], self.rasterBlock_elev[0, 1], self.rasterBlock_elev[0, 2], self.rasterBlock_elev[0, 3],
                     self.rasterBlock_elev[1, 0], self.rasterBlock_elev[1, 1], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[1, 3],
                     self.rasterBlock_elev[2, 0], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[0, 0], self.rasterBlock_elev[2, 3],
                     self.rasterBlock_elev[3, 0], self.rasterBlock_elev[3, 1], self.rasterBlock_elev[3, 2], self.rasterBlock_elev[3, 3]])
            elif self.zone == 4:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[4, 1], self.rasterBlock_x[4, 2], self.rasterBlock_x[4, 3],
                                  self.rasterBlock_x[1, 0], self.rasterBlock_x[1, 1], self.rasterBlock_x[1, 2], self.rasterBlock_x[1, 3],
                                  self.rasterBlock_x[2, 0], self.rasterBlock_x[2, 1], self.rasterBlock_x[4, 0], self.rasterBlock_x[2, 3],
                                  self.rasterBlock_x[3, 0], self.rasterBlock_x[3, 1], self.rasterBlock_x[3, 2], self.rasterBlock_x[3, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[4, 1], self.rasterBlock_y[4, 2], self.rasterBlock_y[4, 3],
                                  self.rasterBlock_y[1, 0], self.rasterBlock_y[1, 1], self.rasterBlock_y[1, 2], self.rasterBlock_y[1, 3],
                                  self.rasterBlock_y[2, 0], self.rasterBlock_y[2, 1], self.rasterBlock_y[4, 0], self.rasterBlock_y[2, 3],
                                  self.rasterBlock_y[3, 0], self.rasterBlock_y[3, 1], self.rasterBlock_y[3, 2], self.rasterBlock_y[3, 3]])
                elev = np.array(
                    [self.rasterBlock_elev[2, 2], self.rasterBlock_elev[4, 1], self.rasterBlock_elev[4, 2], self.rasterBlock_elev[4, 3],
                     self.rasterBlock_elev[1, 0], self.rasterBlock_elev[1, 1], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[1, 3],
                     self.rasterBlock_elev[2, 0], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[4, 0], self.rasterBlock_elev[2, 3],
                     self.rasterBlock_elev[3, 0], self.rasterBlock_elev[3, 1], self.rasterBlock_elev[3, 2], self.rasterBlock_elev[3, 3]])
            if self.zone == 2:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[0, 1], self.rasterBlock_x[0, 2], self.rasterBlock_x[0, 3],
                                  self.rasterBlock_x[1, 4], self.rasterBlock_x[1, 1], self.rasterBlock_x[1, 2], self.rasterBlock_x[1, 3],
                                  self.rasterBlock_x[2, 4], self.rasterBlock_x[2, 1], self.rasterBlock_x[0, 4], self.rasterBlock_x[2, 3],
                                  self.rasterBlock_x[3, 4], self.rasterBlock_x[3, 1], self.rasterBlock_x[3, 2], self.rasterBlock_x[3, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[0, 1], self.rasterBlock_y[0, 2], self.rasterBlock_y[0, 3],
                                  self.rasterBlock_y[1, 4], self.rasterBlock_y[1, 1], self.rasterBlock_y[1, 2], self.rasterBlock_y[1, 3],
                                  self.rasterBlock_y[2, 4], self.rasterBlock_y[2, 1], self.rasterBlock_y[0, 4], self.rasterBlock_y[2, 3],
                                  self.rasterBlock_y[3, 4], self.rasterBlock_y[3, 1], self.rasterBlock_y[3, 2], self.rasterBlock_y[3, 3]])
                elev = np.array(
                    [self.rasterBlock_elev[2, 2], self.rasterBlock_elev[0, 1], self.rasterBlock_elev[0, 2], self.rasterBlock_elev[0, 3],
                     self.rasterBlock_elev[1, 4], self.rasterBlock_elev[1, 1], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[1, 3],
                     self.rasterBlock_elev[2, 4], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[0, 4], self.rasterBlock_elev[2, 3],
                     self.rasterBlock_elev[3, 4], self.rasterBlock_elev[3, 1], self.rasterBlock_elev[3, 2], self.rasterBlock_elev[3, 3]])
            elif self.zone == 3:
                xCoor = np.array([self.rasterBlock_x[2, 2], self.rasterBlock_x[4, 1], self.rasterBlock_x[4, 2], self.rasterBlock_x[4, 3],
                                  self.rasterBlock_x[1, 4], self.rasterBlock_x[1, 1], self.rasterBlock_x[1, 2], self.rasterBlock_x[1, 3],
                                  self.rasterBlock_x[2, 4], self.rasterBlock_x[2, 1], self.rasterBlock_x[4, 4], self.rasterBlock_x[2, 3],
                                  self.rasterBlock_x[3, 4], self.rasterBlock_x[3, 1], self.rasterBlock_x[3, 2], self.rasterBlock_x[3, 3]])
                yCoor = np.array([self.rasterBlock_y[2, 2], self.rasterBlock_y[4, 1], self.rasterBlock_y[4, 2], self.rasterBlock_y[4, 3],
                                  self.rasterBlock_y[1, 4], self.rasterBlock_y[1, 1], self.rasterBlock_y[1, 2], self.rasterBlock_y[1, 3],
                                  self.rasterBlock_y[2, 4], self.rasterBlock_y[2, 1], self.rasterBlock_y[4, 4], self.rasterBlock_y[2, 3],
                                  self.rasterBlock_y[3, 4], self.rasterBlock_y[3, 1], self.rasterBlock_y[3, 2], self.rasterBlock_y[3, 3]])
                elev = np.array(
                    [self.rasterBlock_elev[2, 2], self.rasterBlock_elev[4, 1], self.rasterBlock_elev[4, 2], self.rasterBlock_elev[4, 3],
                     self.rasterBlock_elev[1, 4], self.rasterBlock_elev[1, 1], self.rasterBlock_elev[1, 2], self.rasterBlock_elev[1, 3],
                     self.rasterBlock_elev[2, 4], self.rasterBlock_elev[2, 1], self.rasterBlock_elev[4, 4], self.rasterBlock_elev[2, 3],
                     self.rasterBlock_elev[3, 4], self.rasterBlock_elev[3, 1], self.rasterBlock_elev[3, 2], self.rasterBlock_elev[3, 3]])

        return xCoor, yCoor, elev


class SurfaceArea:

    def __init__(self, rasterBlock_elev, rasterBlock_x, rasterBlock_y, mth, xres, yres):
        self.rasterBlock_elev = rasterBlock_elev # elevation values of a 5*5 matrix
        self.rasterBlock_x = rasterBlock_x # x coordinates of a 5*5 matrix
        self.rasterBlock_y = rasterBlock_y # y coordinates of a 5*5 matrix
        self.mth = mth # method of interpolation
        self.xres = xres # pixel width
        self.yres = yres # pixel height

        if self.mth == 'WA9':
            self.neibrs, self.order = 9, 0 # number of neighboring pixels used in the interpolation, and the order of polynomial

        elif self.mth == 'li':
            self.neibrs, self.order = 3, 1

        elif self.mth == 'biLi4':
            self.neibrs, self.order = 4, 2

        elif self.mth == 'biQuad9':
            self.neibrs, self.order = 9, 4

        elif self.mth == 'biCub16':
            self.neibrs, self.order = 16, 5

    def distance3d(self, xp1, yp1, zp1, xp2, yp2, zp2):
        #Calculate 3D distance between two points
        d = ((xp1 - xp2) ** 2 + (yp1 - yp2) ** 2 + (zp1 - zp2) ** 2) ** 0.5
        return d

    def areatriangle3d(self, x1, y1, z1, x2, y2, z2, x3, y3, z3):
        # Calculate area of a 3D triangle
        a = self.distance3d(x1, y1, z1, x2, y2, z2)
        b = self.distance3d(x2, y2, z2, x3, y3, z3)
        c = self.distance3d(x3, y3, z3, x1, y1, z1)
        # Heron method for calculating the area of a triangle when you know the lengths of all three sides.
        s = (a + b + c) / 2.0
        area = (s * (s - a) * (s - b) * (s - c)) ** 0.5
        return area

    def calculateArea(self):

        sa = 0.0

        # shift the coordinates of central pixel to (0,0) as UTM coordinate are large
        self.rasterBlock_x = self.rasterBlock_x - self.rasterBlock_x[2, 2]
        self.rasterBlock_y = self.rasterBlock_y - self.rasterBlock_y[2, 2]
        xMin = -(self.xres / 2.0)
        xMax = (self.xres / 2.0)
        yMin = -(self.yres / 2.0)
        yMax = (self.yres / 2.0)

        # Points on the boundary of central pixel that their elevation should be interpolated
        # 2    3    4
        # 1&9       5
        # 8    7    6
        pntX = [xMin, xMin, 0, xMax, xMax, xMax, 0, xMin, xMin]
        pntY = [0, yMax, yMax, yMax, 0, yMin, yMin, yMin, 0]

        for i in [1, 2, 3, 4]:  # Dividing the pixel into 4 squares, each of which contains two triangles

            interObject = Interpolation(self.rasterBlock_x, self.rasterBlock_y, self.rasterBlock_elev, i, self.neibrs)

            if self.order == 0:  # IDW

                elev1 = interObject.idw(pntX[(i * 2) - 2], pntY[(i * 2) - 2])
                elev2 = interObject.idw(pntX[(i * 2) - 1], pntY[(i * 2) - 1])
                elev3 = interObject.idw(pntX[(i * 2)], pntY[(i * 2)])

            else: # polynomial
                m = interObject.polyfit2d(self.order)
                elev1 = interObject.polyval2d(pntX[(i * 2) - 2], pntY[(i * 2) - 2], m)
                elev2 = interObject.polyval2d(pntX[(i * 2) - 1], pntY[(i * 2) - 1], m)
                elev3 = interObject.polyval2d(pntX[(i * 2)], pntY[(i * 2)], m)

            a1 = self.areatriangle3d(self.rasterBlock_x[2, 2], self.rasterBlock_y[2, 2], self.rasterBlock_elev[2, 2], pntX[(i * 2) - 2],
                                pntY[(i * 2) - 2], elev1, pntX[(i * 2) - 1], pntY[(i * 2) - 1], elev2)
            a2 = self.areatriangle3d(self.rasterBlock_x[2, 2], self.rasterBlock_y[2, 2], self.rasterBlock_elev[2, 2], pntX[(i * 2) - 1],
                                pntY[(i * 2) - 1], elev2, pntX[(i * 2)], pntY[(i * 2)], elev3)

            sa = sa + (a1 + a2)

        return sa

def  par(A):
    '''
    This function receives 5 rows of a numpy array
    and calculate surface area for all pixels of the middle row
    '''

    _, arr, X, Y, mth, xres, yres = A

    SA = []
    for j in np.arange(2, np.size(arr, 1) - 2):

        # Extract the values of a 5 by 5 moving window
        rasterBlock_elev = arr[0:6, j - 2:j + 3]  # this 5 by 5 matrix would contain the elevation of 25 neighbor pixels
        rasterBlock_x = X[0:6, j - 2:j + 3]  # this 5 by 5 matrix would contain the x coordinate of 25 neighbor pixels
        rasterBlock_y = Y[0:6, j - 2:j + 3]  # this 5 by 5 matrix would contain the y coordinate of 25 neighbor pixels

        # Check for no data values
        if np.any(rasterBlock_elev < -1000):
            SA.append(np.nan)
        else:
            # Calculate surface area for different methods using the SurfaceArea class
            objArea = SurfaceArea(rasterBlock_elev, rasterBlock_x, rasterBlock_y, mth, xres, yres)
            SA.append(objArea.calculateArea())

    return SA

if __name__ == '__main__':

    # Get the inputs (1- input folder, 2- output folder, 3- number of CPUs used in multiprocessing) from the user
    # to run the code, you can try
    # python surfacearea.py -i './surface-area-computations/input' -o './surface-area-computations/output' -cpu 5
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', help="Identify the input directory including all of the DEMs", required=True)
    parser.add_argument('-o', '--output', help="Identify the output directory to save the surface area rasters",
                        required=True)
    parser.add_argument('-cpu', '--cpu', help="Identify the number of CPUs used for multiprocessing", required=True,
                        type=int)
    args = parser.parse_args()

    if os.path.isdir(args.input) and os.path.isdir(args.output):
        input = args.input # Set the current workspace
        output = args.output # Results are saved in the following directory
    else:
        raise argparse.ArgumentTypeError(f"input or output is not a valid path")

    if isinstance(args.cpu, numbers.Integral):
        nbrOfCpus = args.cpu # Set the number of CUPs
    else:
        raise argparse.ArgumentTypeError(f"Number of CUPs should be an integer number")


    # find all of the DEM files in the input folder
    DEMs = []
    for root, dirs, files in os.walk(input):
        for file in files:
            if file.endswith(".tif"):
                DEMs.append(file)

    # methods used for calculating surface area
    methods = ['WA9', 'li', 'biLi4', 'biQuad9', 'biCub16']

    # Declare variables to keep track of time for each method to compare the efficiency of each method
    Time = namedtuple('Time', methods)

    for dem in DEMs:  # surface area rasters are calculated for each input DEM
        print(f'Calculating surface area for {dem} ...')
        # create a raster object to get its properties and numpy arrays
        inputRasterObj = Raster(f'{input}/{dem}')
        xres, yres = inputRasterObj.xres, inputRasterObj.yres
        yres = abs(yres)  # yres is neative by default
        X, Y, arr = inputRasterObj.raster2Numpy()

        for mth in methods: # surface area rasters are calculated for each interpolation method
            print(f'Calculating surface area for method {mth} ...')
            tmp = time()  # Keep track of time for each method.
            saRasters = arr*np.nan # surface area raster

            p = multiprocessing.Pool(processes=nbrOfCpus)
            a = p.map(par, [(i, arr[i - 2:i + 3, :], X[i - 2:i + 3, :], Y[i - 2:i + 3, :], mth, xres, yres) for i in np.arange(2, np.size(arr, 0) - 2)])
            saRasters = np.array(a)
            p.close()
            p.join()

            if dem == DEMs[0]: setattr(Time, mth, (time() - tmp))

            # Converting surface area numpy to a geotif using the Numpy2Raster method in the Raster class
            if os.path.exists(output + '/SA_' + mth + str(int(xres))):
                os.remove(output + '/SA_' + mth + str(int(xres)))
            inputRasterObj.numpy2Raster(saRasters, output + '/SA_' + mth + str(int(xres)))

    for mth in methods:
        print ("Processing time for " + mth + "is: " + str(getattr(Time, mth)))