# -*- coding: utf-8 -*-
'''----------------------------------------------------------------------------------
File Name      : main.py
Author         : Mehran Ghandehari
Organization   : University of Colorado at Boulder
Created        : Feb. 02th, 2018
Python Version : 2.7

-- Description --
Surface-adjusted area is calculated in this code using different interpolation
methods and different contiguity configurations across different resolutions
Note1: Lidar data (30 DEM) is used as benchmark
Note2: Jenness's method for calculating surface area is implemented and compared with the proposed methods
Jenness, J.S., 2004. Calculating landscape surface area from digital elevation models. Wildlife Society Bulletin, 32(3), pp.829-839.
----------------------------------------------------------------------------------'''
# Import modules
import multiprocessing
import rasterio
import os
import numpy as np
from time import time
temp = time()

# Import my modules
import raster2Numpy  # Used to covert a raster files to a numpy array
import numpy2Raster  # Used to covert a numpy array to a raster tif file
import parallel
import chunkBenchmark

# This functin create a list of tubples; this is used to generate list of range tuples with given boundaries
def gen_range(start, stop, step):
    while start < stop:
        yield (start, min(start + step, stop))
        start = start + step

if __name__ == '__main__':

    # Set the current workspace
    os.chdir(r"C:\Users\Administrator\Desktop\python_V6.1\data")

    # Results are saved in the following directory
    output = r"C:\Users\Administrator\Desktop\python_V6.1\Results"
    if os.path.isdir(output) == False:
        print("The output directoru does not exist")

    # Input data (DEMs)
    benchmark = 'dem3m'
    DEMs = ['dem10m', 'dem30m', 'dem100m', 'dem1000m']

    # methods used for calculating surface area
    #['Jenness', 'ben', 'WA9', 'li', 'biLi4', 'biQuad9', 'biCub16']
    methods = ['Jenness', 'ben', 'slp', 'WA9', 'li', 'biLi4', 'biQuad9', 'biCub16']

    # Declare vaqriable to keep track of time for each method
    for mth in methods:
        globals()['time' + mth] = 0

    # Open the lidar data as a benchmark
    X_lidar, Y_lidar, arr_lidar, xres_lidar, yres_lidar, minX_lidar, minY_lidar, maxX_lidar, maxY_lidar, sourceProj_lidar = raster2Numpy.convert(benchmark)
    lidarProperties = [X_lidar, Y_lidar, arr_lidar, xres_lidar, yres_lidar,minX_lidar, minY_lidar, maxX_lidar, maxY_lidar]


    for dem in DEMs:  # for each resolution surface area rasters are calculated for various methods

        X, Y, arr, xres, yres, minX, minY, maxX, maxY, sourceProj = raster2Numpy.convert(dem) # Read in the DEM and its properties
        yres = abs(yres) # yres is neative


        chunck = [x for x in gen_range(2, np.size(arr, 0) - 2, 10000)] # (start, stop, step); step is the numbers of rows processed in the parrallel processing

        for mth in methods:
            print (mth)
            tmp = time()  # Keep track of time for each method. This is done only for 10m DEM
            saRasters = arr*np.nan

            if mth == 'ben':

                # Find the corresponding row of benchmark raster for each DEM row
                lidarValues = np.zeros((np.size(arr, 0), 3)).astype(int)
                for i in np.arange(0, np.size(arr, 0)):
                    lidarValues[i, :] = chunkBenchmark.find(X[i, 0], Y[i, 0], yres, lidarProperties)

                for k1, k2 in chunck:
                    print (k1)
                    p = multiprocessing.Pool(processes=32)
                    a = p.map(parallel.par,
                              [(i, arr[i - 2:i + 3, :], X[i - 2:i + 3, :], Y[i - 2:i + 3, :], mth, xres, yres, [arr_lidar[lidarValues[i,0]:lidarValues[i,1]+1,:], xres_lidar, yres_lidar, minX_lidar, lidarValues[i,2], lidarValues[i,0]]) for i in np.arange(k1, k2)])
                    saRasters[k1:k2, 2:np.size(arr, 1) - 2] = np.array(a)
                    p.close()
                    p.join()

            else:

                for k1, k2 in chunck:
                    print (k1)
                    p = multiprocessing.Pool(processes=32)
                    a = p.map(parallel.par,
                              [(i, arr[i - 2:i + 3, :], X[i - 2:i + 3, :], Y[i - 2:i + 3, :], mth, xres, yres,[0,0,0,0,0,0]) for i in np.arange(k1, k2)])
                    saRasters[k1:k2, 2:np.size(arr, 1) - 2] = np.array(a)
                    p.close()
                    p.join()

            if dem == DEMs[0]: globals()['time' + mth] = globals()['time' + mth] + (time() - tmp)

            # Converting surface area numpies to a geotif using the Numpy2Raster module
            if os.path.exists(output + '\SA_' + mth + str(int(xres))):
                os.remove(output + '\SA_' + mth + str(int(xres)))
            #numpy2Raster.convert(saRasters[mth], minX, minY, maxX, maxY, xres, sourceProj, output + '\SA_' + mth + str(int(xres)))
            numpy2Raster.convert(saRasters, minX, minY, maxX, maxY, xres, sourceProj,output + '\SA_' + mth + str(int(xres)))

    for mth in methods:
        print ("Processing time for " + mth + "is: " + str(globals()['time' + mth]))