import numpy as np
import cv2 
import time
import math
from motor_control import forward,reverse,pivot_left,pivot_right


def show(img):
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow('output', 600,600)
    cv2.imshow('output',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([58, 48, 85], dtype="uint8")
    upper_green = np.array([93, 174, 232], dtype="uint8")
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,np.array([]), minLineLength=5, maxLineGap=150)
    return line_segments

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    if slope == 0:
        slope = 0.1
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        print("no line segments detected")
        return lane_lines
    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines")
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))
    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    return lane_lines


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90
    #print(steering_angle)
    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    return heading_image

def deviation(steering_angle):
    deviation = steering_angle - 90
    error = abs(deviation)
    return error


def main(img):
    edges = detect_edges(img)
    cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(img,line_segments)
    steering_angle = get_steering_angle(img,lane_lines)
    heading_image = display_heading_line(img,steering_angle)
    points = display_lines(img,lane_lines)
    heading_line = cv2.bitwise_or(heading_image, points)
    error = deviation(steering_angle)
    return heading_line,len(lane_lines),steering_angle,error
two_lane=0
count=0

cam = cv2.VideoCapture(0)
width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
height_h = int(0.5*height)

fourcc = cv2.VideoWriter_fourcc(*'XVID') 
out = cv2.VideoWriter('/home/nitin/Desktop/Jetson_Hardware/Road_Follower/frame.avi', fourcc, 20.0, (width, height))
out_1 = cv2.VideoWriter('/home/nitin/Desktop/Jetson_Hardware/Road_Follower/frame_1.avi', fourcc, 20.0, (width, height))

while True:
    _,frame=cam.read()
    out.write(frame)
    
    img,lane_lines,steering_angle,error = main(frame)
    print("Lane Lines||||{} Steering Angle||||{} Error||||{}".format(lane_lines,steering_angle,error))
    out_1.write(img)
    
    if lane_lines==2 and steering_angle<100:
        forward(0.03)
        time.sleep(0.03)

    elif lane_lines==2 and steering_angle>100:
        pivot_right(0.03)
        time.sleep(0.03)
      
                  
    elif lane_lines==1 and steering_angle>100:
       pivot_right(0.07)
       time.sleep(0.07)
        
          
    elif lane_lines==1 and steering_angle<100:
        pivot_left(0.1)
        time.sleep(0.1)
     

    cv2.imshow('img',img)
    cv2.moveWindow('img',0,0)
  
    
    if cv2.waitKey(1)==ord('x'):
        break

out.release()
out_1.release()
cam.release()
cv2.destroyAllWindows()