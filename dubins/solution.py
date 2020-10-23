#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Sean Johan M Deloddere
# {student id} ?
# seandel@kth.se

import sys
from dubins import Car, step
from math import sqrt,pi
from numpy import array,zeros,set_printoptions,rad2deg, arctan2
#import matplotlib.pyplot as plt

steps = 5


class Point():
    def __init__(self, x=None, y=None, prev=None):
        self.x = x
        self.y = y
        self.prev = prev

        self.distance = 0
        self.heur = 0
        self.total = 0

    def __eq__(self, other):
        same = (self.x == other.x and self.y == other.y)
        return same


def heur(x,y,car):
    return sqrt((x-steps*car.xt)**2+(y-steps*car.yt)**2)


def dist(point):
    distance = 0
    if point.x == point.prev.x or point.y == point.prev.y:
        distance = 1
    else:
       distance = 1.4 #sqrt(2)
    return distance


def make_map(car):
    #make map with "steps" points for every 1 meter
    map = zeros((int(steps*car.yub+1),int(steps*car.xub+1)))
    #map[:, 0] =  1
    #map[:, int(steps*car.xub)] =  1
    #map[0, :] =  1
    #map[int(steps*car.yub), :] =  1
    #map[int(steps*car.yub)-int(car.y0*steps),int(car.x0*steps)] = 0
    #map[int(steps*car.yub)-int(car.yt*steps),int(car.xt*steps)] = 0

    #fill in obstacles
    for i in range(int(steps*car.yub+1)):
        for j in range(int(steps*car.xub+1)):
            for obstacle in car.obs:
                if sqrt((j-obstacle[0]*steps)**2+(i-obstacle[1]*steps)**2) < obstacle[2]*steps+0.2:
                    map[i,j]=2
                elif sqrt((j-obstacle[0]*steps)**2+(i-obstacle[1]*steps)**2) < obstacle[2]*steps+2.2:
                    map[i,j]=1
        


    #set_printoptions(threshold=sys.maxsize)
    #print(map)
    return map



def astar(map, car):
    # Create begin and end point
    begin_point = Point(steps*car.x0, steps*car.y0, None)
    begin_point.distance = begin_point.heur = begin_point.total = 0
    end_point = Point(steps*car.xt, steps*car.yt, None)
    end_point.distance = end_point.heur = end_point.total = 0

    #print(end_point.x,end_point.y,map[int(end_point.y)][int(end_point.x)])

    # Initialize checked and unchecked list
    unchecked = []
    checked = []

    # Add the begin point
    unchecked.append(begin_point)

    # Loop until unchecked list is empty or you end is on top of unchecked list
    while len(unchecked) > 0:
        print("len unchecked: ",len(unchecked))

        # Get the current point from unchecked list
        current_point = unchecked[0]
        current_index = 0
        for index, item in enumerate(unchecked):
            if item.total < current_point.total:
                current_point = item
                current_index = index

        #print("x: ",current_point.x)
        #print("y: ",current_point.y)

        # Pop current off unchecked list, add to checked list
        unchecked.pop(current_index)
        checked.append(current_point)
        #print("after pop: ",len(unchecked))
       

        # Found end point
        if current_point == end_point:
            #print("found goal")
            path = []
            distances = []
            current = current_point
            while current is not None:
                path.append((current.x/steps, current.y/steps))
                distances.append((current.distance))
                current = current.prev
            return path[::-1] # Return reversed path, checked list and distances in path

        # Generate next_points
        next_points = []
        for direction in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
        #for direction in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, 1), (1, -1), (1, 1)]:
            
            # Get coordinates of next points
            point_x = current_point.x + direction[0]
            point_y = current_point.y + direction[1]

            # check if its within the map
            if point_y > (len(map) - 1) or point_y < 0 or point_x > (len(map[len(map)-1]) -1) or point_x < 0:
                continue

            # check if there is not an obstacle
            if map[int(point_y)][int(point_x)] != 0.0:
                continue

            # Create new point
            new_point = Point(point_x, point_y, current_point)

            #if new_point == end_point:
                #print("***********NEXT=END**********")

            # check if point is in checked already
            if new_point in checked:
                #print(direction, "got out")
                continue

            # Append the new point
            #print(direction, "didnt get out")
            next_points.append(new_point)

        #i=0
        # Loop through next_points
        for next in next_points:
            
            # next is in the checked list
            for checked_next in checked:
                if next == checked_next:
                    #print("next is checked")
                    break
            else:
                # Calculate astar value
                next.distance = current_point.distance + dist(next)
                next.heur = heur(next.x,next.y,car)
                next.total = next.distance + next.heur

                # next is in the unchecked list
                for unchecked_point in unchecked:
                    if next == unchecked_point and next.distance >= unchecked_point.distance:
                        #print("next is unchecked")
                        break
                else:
                    # Add the next to the unchecked list
                    unchecked.append(next)


def angle_between(p1, p2):
    return arctan2((p2[1]-p1[1]),(p2[0]-p1[0]))


def distance_between_points(p1,p2):
    return sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))


def solution(car):

    ''' <<< write your code below >>> '''
    print(" ")
    print("*****************************************************************NEW CASE*****************************************************************************")
    map = make_map(car)
    path = astar(map, car)
    print("PATH: ")
    print(path)
    #for tupel in path:
    #    if map[int(tupel[0]),int(tupel[1])] == 1:
    #        print("you messed up bro")
    #print("no collisions")
    theta = 0
    phi = 0
    t = 0
    xn = car.x0
    yn = car.y0
    list_for_phi = []
    list_for_t = [0]
    #angle constant
    c = 0.1

    actual_x = []
    actual_y = []

    counter = 0

    for x in range(len(path)):
        #print(" ")
        #print(' <<<<<<<<<<<<<<<<<<<<<<<<<<<< OUTSIDE WHILE LOOP >>>>>>>>>>>>>>>>>>>>>>>>>>')
        #angle_between_two_points = angle_between((xn, yn), path[x+2])
        #print('current point: (', xn,yn,')', 'next point:',path[x],  'angle_between_in_degrees:',rad2deg(angle_between_two_points))
        #print(' DISTANCE LEFT:', round(distance_between_points([xn,yn],path[x]),1))

        while distance_between_points([xn,yn],path[x]) > 0.2:
            #print(' <<<<<<<<<<<<<<<<<<<<<<<<<<<< INSIDE WHILE LOOP >>>>>>>>>>>>>>>>>>>>>>>>>>')
            if xn>=car.xub or yn>car.yub:
                print("YOU DONE FUCKED UP:", xn, yn)
                break

            angle_between_two_points = angle_between((xn, yn), path[x])
  
            #if distance_between_points([xn,yn],path[len(path)-1]) < 1 or (xn > path[x][0] and yn > path[x][1]):
            #    if (path[x+1][0] > path[x][0] or path[x+1][1] > path[x][1]) and (path[x+2][0] > path[x+1][0] or path[x+2][1] > path[x+1][1]):
            #        break    # break here
            #    else:
            #        print("going left down")
            #        if distance_between_points([xn,yn],path[len(path)-1]) < 1 or (xn < path[x][0] and yn < path[x][1]):
            #            break

            #if distance_between_points([xn,yn],path[len(path)-1]) < 1:
            #    break


            if round(distance_between_points([xn,yn],path[x-1]),1) < 0.9:
                break
            
            #if (xn > path[x+2][0] and yn > path[x+2][1]):
            #   if counter <= 8:
            #        counter += 1
            #        print(counter)
            #        break
            #   else:
            #       if thetan > -pi/2:
            #           if (xn > path[x+2][0] and yn < path[x+2][1]):
            #               break
            #       else:
            #           if (xn < path[x+2][0] and yn < path[x+2][1]):
            #               break
            #else:
            #    counter = 0


            #print('current point: (', xn,yn,')', 'next point:',path[x],  'angle_between_in_degrees:', rad2deg(angle_between_two_points))
            #if (xn > path[x+2][0] and yn > path[x+2][1]):
            #    phi = -phi
            xn, yn, thetan = step(car, xn, yn, theta, phi)
            relative_angle = angle_between_two_points - thetan
            
            #print('rel_ang:', relative_angle)
            #print('thetan:', thetan)
            #print('angle_between:', angle_between_two_points)
            #if abs(angle_between_two_points - thetan) <= 0.001:
            if relative_angle <= c and relative_angle >= - c:
                phi = 0
                theta = thetan
                #print('phi kept the same,headline_result:',abs(relative_angle - relative_angle))
            elif relative_angle   < -c:
                phi = -pi/4
                theta = thetan
                #print('phi -45 degrees:', theta, 'headline_result:',relative_angle - relative_angle)

            elif relative_angle > c:
                phi = pi/4
                theta = thetan
                #print('phi +45 degrees:', theta, 'headline_result:',relative_angle - relative_angle)
            else:
                #print('else')
                phi = pi/4
                theta = thetan
            print(' ')
            actual_x.append(xn)
            actual_y.append(yn)
            #print('final point reached:', round(xn,4), round(yn,4), '///distance left ',round(distance_between_points([xn,yn],path[x+2]),1))
            t += 0.01
            list_for_phi.append(phi)
            list_for_t.append(t)

            #if theta == 0:
            #    if xn > path[x][0]:
            #        break
            #elif theta == pi/2:
            #    if yn > path[x][1]:
            #        break
            #elif theta == pi:
            #    if xn < path[x][0]:
            #        break
            #elif theta == -pi/2:
            #    if yn < path[x][1]:
            #        break
            #elif theta >  0 and theta < pi/2:
            #    if (xn > path[x][0] and yn > path[x][1]):
            #        break
            #elif theta >  pi/2 and theta < pi:
            #    if (xn < path[x][0] and yn > path[x][1]):
            #        break
            #elif theta <  0 and theta > -pi/2:
            #    if (xn > path[x][0] and yn < path[x][1]):
            #        break
            #elif theta <  -pi/2 and theta > -pi:
            #    if (xn < path[x][0] and yn < path[x][1]):
            #        break

        print (' ')
        print (' -----------------------------------------------------------------------------------------')
        
       # if t > 0.99:
        #    break    # break here


    #print('Out of loop')
    
    controls, times = list_for_phi,  list_for_t
    
    #print(controls)
    obstx = []
    obsty = []
    actual_obstx = []
    actual_obsty = []

    for i in range(int(steps*car.yub+1)):
        for j in range(int(steps*car.xub+1)):
            if map[i][j] == 2:
                actual_obstx.append(j/steps)
                actual_obsty.append(i/steps)
            elif map[i][j] == 1:
                obstx.append(j/steps)
                obsty.append(i/steps)
            

    path_list_x = []
    path_list_y = []
    for x in range(len(path)):
        path_list_x.append(path[x][0])
        path_list_y.append(path[x][1])

    #plt.scatter(actual_x, actual_y,c='r')
    #plt.scatter(path_list_x,path_list_y)
    #plt.scatter(obstx,obsty)
    #plt.scatter(actual_obstx,actual_obsty)
    #plt.show()


    print("======================================================================================================================================================")
    print(" ")
    return controls, times

