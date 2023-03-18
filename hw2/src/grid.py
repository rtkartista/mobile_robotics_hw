#! /usr/bin/env python3
import pdb
import numpy as np
import cv2
class grid:
    def __init__(self, x, y, resolution, prior):
        self.res = resolution
        self.Pinit = prior
        self.xlim = x/2
        self.ylim =y/2

        # fill value is always kept below 1
        # Log value is used to achieve this result  
        self.x = np.arange(start = -x/2, stop = x/2 + resolution, step = resolution)
        self.y = np.arange(start = -y/2, stop = y/2 + resolution, step = resolution)
        self.grid_probabilty = np.full(shape = (len(self.x), len(self.y)), fill_value = self.get_log_odds(self.Pinit))
    
    # Why are log_odds implemented
    # symmetric
    # centered around zero, which makes their sign (+/-) interpretable,
    # linear
    # can be easily transformed to probabilities
    def get_log_odds(self, p):
        return np.log(p / (1 - p))

    def get_prob(self, l):
        return 1 - 1 / (1 + np.exp(l))
    
    # left bottom - image origin
    # world - (0,0)
    def xy_pix(self, x, y):
        x = int((x + self.xlim)/self.res)
        y = int((y + self.ylim)/self.res)
        if x>=len(self.x)-1:
            x = len(self.x)-2
        if y>=len(self.x)-1:
            y = len(self.x)-2
        return (len(self.x)-2-y,x)
    
    def update(self, x, y, p):
		# update probability matrix using inverse sensor model
        self.grid_probabilty[x][y] += self.get_log_odds(p)

    # line extraction from bresenham algorithm
    def get_laser_line(self, x1, y1, x2, y2):
        # Output pixels
        X_bres = []
        Y_bres = []

        x = x1
        y = y1
        
        dx = np.abs(x2 - x1)
        dy = np.abs(y2 - y1)
        
        s_x = np.sign(x2 - x1)
        s_y = np.sign(y2 - y1)

        # check if slope more than 1
        if dy > dx:
            dx, dy = dy, dx
            interchange = True
        else:
            interchange = False
        
        # calculate decision parameters
        pk = 2 * dy - dx

        # calculate pk increment parameters
        pk1 = 2 * dy
        pk2 = 2 * (dy - dx)

        # append the output pixels
        X_bres.append(x)
        Y_bres.append(y)

        # point (x2,y2) must not be included
        # for each intermediate integers btw x1 and x2
        for i in range(1, dx):
            if pk < 0:
                if interchange:
                    y += s_y
                else:
                    x += s_x
                pk = pk + pk1
            else:
                y += s_y
                x += s_x
                pk = pk + pk2
            # mark output pixels
            X_bres.append(x)
            Y_bres.append(y)
        return zip(X_bres, Y_bres)

    def map_updated_grid(self, x0, y0, arrX, arrY):
        # grayscale image
        gray_image = 1 - self.get_prob(self.grid_probabilty)

        # repeat values of grayscale image among 3 axis to get BGR imagw
        rgb_image = np.repeat(a = gray_image[:,:,np.newaxis], 
                            repeats = 3,
                            axis = 2)
        
        # setting robot pose BLUE
        rgb_image[x0, y0, 0] = 0.0
        rgb_image[x0, y0, 1] = 0.0
        rgb_image[x0, y0, 2] = 1.0

        # occupied spots GREEN
        for (x, y) in zip(arrX,arrY):
            rgb_image[x, y, 0] = 0.0
            rgb_image[x, y, 1] = 1.0
            rgb_image[x, y, 2] = 0.0

        resized_image = cv2.resize(src = rgb_image, 
                        dsize = (1000, 1000), 
                        interpolation = cv2.INTER_AREA)

        rotated_image = cv2.rotate(src = resized_image, 
                        rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

        return resized_image