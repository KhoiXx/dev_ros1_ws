import math
from PVector import PVector
from UtilitiesMacroAndConstant import *


class Vehicle:
    def __init__(self, *args, **kwargs):
        self.ang_front = 0
        self.ang_front = 0
        self.ang_rear = 0
        self.angle = 0
        self.len = 0
        self.center = PVector()

        # Store the previous angular
        self.prev_front_left_ang = 0
        self.prev_front_right_ang = 0
        self.prev_rear_left_ang = 0
        self.prev_rear_right_ang = 0

        # Initailize steering value
        self.steering_front_left = 0
        self.steering_front_right = 0
        self.steering_rear_left = 0
        self.steering_rear_right = 0

        # Scale from pixel to real value
        self.unit = 0

        # Instant target
        self.rear_center_target = PVector()
        self.front_center_target = PVector()
        self.front_left_target = PVector()
        self.front_right_target = PVector()
        self.rear_left_target = PVector()
        self.rear_right_target = PVector()

        # Real vector
        self.rear_center = PVector()
        self.front_center = PVector()
        self.front_left = PVector()
        self.front_right = PVector()
        self.rear_left = PVector()
        self.rear_right = PVector()

        # Real velocity
        self.vel_rear_left = PVector.sub2Vectors(self.rear_left_target, self.rear_left)
        self.vel_rear_right = PVector.sub2Vectors(self.rear_right_target, self.rear_right)
        self.vel_rear_center = PVector.sub2Vectors(self.rear_center_target, self.rear_center)
        self.vel_front_center = PVector.sub2Vectors(self.front_center_target, self.front_center)
        self.vel_front_right = PVector.sub2Vectors(self.front_right_target, self.front_right)
        self.vel_front_left = PVector.sub2Vectors(self.front_left_target, self.front_left)

        # Real frame left and right
        self.temporary_left_frame = PVector.sub2Vectors(self.front_left, self.rear_left)
        self.temporary_right_frame = PVector.sub2Vectors(self.front_right, self.rear_right)

        if len(args) == 3:
            self.center = PVector(args[0], args[1])
            self.len = args[2]
            self.angle = 0
            self.update()
            self.init_real_points()

    def init_real_points(self):
        '''
        Initialize real points according to the first tate of MR
        :return:
        '''
        self.vel_rear_center = PVector.sub2Vectors(self.rear_center_target, self.rear_center)
        self.vel_front_center = PVector.sub2Vectors(self.front_center_target, self.front_center)

        self.vel_rear_left = PVector.sub2Vectors(self.rear_left_target, self.rear_left)
        self.vel_rear_right = PVector.sub2Vectors(self.rear_right_target, self.rear_right)
        self.vel_front_right = PVector.sub2Vectors(self.front_right_target, self.front_right)
        self.vel_front_left = PVector.sub2Vectors(self.front_left_target, self.front_left)

        self.rear_center.add(self.vel_rear_center)
        self.front_center.add(self.vel_front_center)

        self.front_left.add(self.vel_front_left)
        self.front_right.add(self.vel_front_right)
        self.rear_left.add(self.vel_rear_left)
        self.rear_right.add(self.vel_rear_right)

    def calculate_rear_center(self):
        dx = - self.len * math.cos(self.angle)
        dy = - self.len * math.sin(self.angle)
        self.rear_center_target.set(self.center.x + dx, self.center.y + dy)

    def calculate_front_center(self):
        dx = + self.len * math.cos(self.angle)
        dy = + self.len * math.sin(self.angle)
        self.front_center_target.set(self.center.x + dx, self.center.y + dy)

    def calculate_center_point(self):
        dx = + self.len * math.cos(self.angle)
        dy = + self.len * math.sin(self.angle)

    def calculate_front_left_target(self):
        temp_angle = self.angle - ROBOT_ANGLE
        dx = ROBOT_DISTANCE_FROM_CENTER * math.cos(temp_angle)
        dy = ROBOT_DISTANCE_FROM_CENTER * math.sin(temp_angle)
        self.front_left_target.set(self.center.x + dx, self.center.y + dy)

    def calculate_front_right_target(self):
        temp_angle = self.angle + ROBOT_ANGLE
        dx = ROBOT_DISTANCE_FROM_CENTER * math.cos(temp_angle)
        dy = ROBOT_DISTANCE_FROM_CENTER * math.sin(temp_angle)
        self.front_right_target.set(self.center.x + dx, self.center.y + dy)

    def calculate_rear_left_target(self):
        temp_angle = self.angle + ROBOT_ANGLE - math.pi
        dx = ROBOT_DISTANCE_FROM_CENTER * math.cos(temp_angle)
        dy = ROBOT_DISTANCE_FROM_CENTER * math.sin(temp_angle)
        self.rear_left_target.set(self.center.x + dx, self.center.y + dy)

    def calculate_rear_right_target(self):
        temp_angle = self.angle - ROBOT_ANGLE - math.pi
        dx = ROBOT_DISTANCE_FROM_CENTER * math.cos(temp_angle)
        dy = ROBOT_DISTANCE_FROM_CENTER * math.sin(temp_angle)
        self.rear_right_target.set(self.center.x + dx, self.center.y + dy)

    def update(self):
        '''
        Update all target position of the vehicle
        :return:
        '''
        self.calculate_front_left_target()
        self.calculate_front_right_target()
        self.calculate_rear_left_target()
        self.calculate_rear_right_target()
        self.calculate_rear_center()
        self.calculate_front_center()

    def follow(self, tx, ty):
        target = PVector(tx, ty)
        dislocation = PVector.sub2Vectors(target, self.center)
        accel = dislocation.copy()
        accel.setMag(accel.mag() / ROBOT_SCALE)

        # print("Important: ", accel.mag())
        self.angle = dislocation.heading()
        # print("Important: ", self.angle)
        self.center.add(accel)

        self.update()

        self.vel_rear_left = PVector.sub2Vectors(self.rear_left_target, self.rear_left)
        self.vel_rear_right = PVector.sub2Vectors(self.rear_right_target, self.rear_right)
        self.vel_rear_center = PVector.sub2Vectors(self.rear_center_target, self.rear_center)
        self.vel_front_center = PVector.sub2Vectors(self.front_center_target, self.front_center)
        self.vel_front_right = PVector.sub2Vectors(self.front_right_target, self.front_right)
        self.vel_front_left = PVector.sub2Vectors(self.front_left_target, self.front_left)

        # Update the frame
        self.temporary_left_frame = PVector.sub2Vectors(self.front_left, self.rear_left)
        self.temporary_right_frame = PVector.sub2Vectors(self.front_right, self.rear_right)

        self.vel_rear_center.setMag(self.vel_rear_center.mag() / SCALE)
        self.rear_center.add(self.vel_rear_center)

        self.vel_front_center.setMag(self.vel_front_center.mag() / SCALE)
        self.front_center.add(self.vel_front_center)

        self.vel_front_left.setMag(self.vel_front_left.mag() / SCALE)
        self.front_left.add(self.vel_front_left)

        self.vel_front_right.setMag(self.vel_front_right.mag() / SCALE)
        self.front_right.add(self.vel_front_right)

        self.vel_rear_left.setMag(self.vel_rear_left.mag() / SCALE)
        self.rear_left.add(self.vel_rear_left)

        self.vel_rear_right.setMag(self.vel_rear_right.mag() / SCALE)
        self.rear_right.add(self.vel_rear_right)

        self.steering_front_left = self.vel_front_left.heading() - self.prev_front_left_ang
        self.steering_front_right = self.vel_front_right.heading() - self.prev_front_right_ang
        self.steering_rear_left = self.vel_rear_left.heading() - self.prev_rear_left_ang
        self.steering_rear_right = self.vel_rear_right.heading() - self.prev_rear_right_ang

        self.prev_front_left_ang = self.vel_front_left.heading()
        self.prev_front_right_ang = self.vel_front_right.heading()
        self.prev_rear_left_ang = self.vel_rear_left.heading()
        self.prev_rear_right_ang = self.vel_rear_right.heading()

    def follow_vector(self, target_vector):
        accel = target_vector.copy()
        # mag unchanged
        accel.setMag(accel.mag())

        self.angle = target_vector.heading()
        self.center.add(accel)

        self.update()

        self.vel_rear_left = PVector.sub2Vectors(self.rear_left_target, self.rear_left)
        self.vel_rear_right = PVector.sub2Vectors(self.rear_right_target, self.rear_right)
        self.vel_rear_center = PVector.sub2Vectors(self.rear_center_target, self.rear_center)
        self.vel_front_center = PVector.sub2Vectors(self.front_center_target, self.front_center)
        self.vel_front_right = PVector.sub2Vectors(self.front_right_target, self.front_right)
        self.vel_front_left = PVector.sub2Vectors(self.front_left_target, self.front_left)

        # Update the frame
        self.temporary_left_frame = PVector.sub2Vectors(self.front_left, self.rear_left)
        self.temporary_right_frame = PVector.sub2Vectors(self.front_right, self.rear_right)

        self.vel_rear_center.setMag(self.vel_rear_center.mag() / SCALE)
        self.rear_center.add(self.vel_rear_center)

        self.vel_front_center.setMag(self.vel_front_center.mag() / SCALE)
        self.front_center.add(self.vel_front_center)

        self.vel_front_left.setMag(self.vel_front_left.mag() / SCALE)
        self.front_left.add(self.vel_front_left)

        self.vel_front_right.setMag(self.vel_front_right.mag() / SCALE)
        self.front_right.add(self.vel_front_right)

        self.vel_rear_left.setMag(self.vel_rear_left.mag() / SCALE)
        self.rear_left.add(self.vel_rear_left)

        self.vel_rear_right.setMag(self.vel_rear_right.mag() / SCALE)
        self.rear_right.add(self.vel_rear_right)

        self.steering_front_left = self.vel_front_left.heading() - self.prev_front_left_ang
        self.steering_front_right = self.vel_front_right.heading() - self.prev_front_right_ang
        self.steering_rear_left = self.vel_rear_left.heading() - self.prev_rear_left_ang
        self.steering_rear_right = self.vel_rear_right.heading() - self.prev_rear_right_ang

        self.prev_front_left_ang = self.vel_front_left.heading()
        self.prev_front_right_ang = self.vel_front_right.heading()
        self.prev_rear_left_ang = self.vel_rear_left.heading()
        self.prev_rear_right_ang = self.vel_rear_right.heading()

    def get_next_angle(self, degree = True):
        '''
        Get next robot steering angle

        :param: degree if True will return angle in degree else return angle in radian

        :return: [front_left_angle, front_right_angle, rear_left_angle, rear_right_angle]
        '''

        tmp_left_frame = self.temporary_left_frame.heading()
        tmp_right_frame = self.temporary_right_frame.heading()

        front_left_angle = (self.vel_front_left.heading() - tmp_left_frame)
        front_right_angle = (self.vel_front_right.heading() - tmp_right_frame)
        rear_left_angle = (self.vel_rear_left.heading() - tmp_left_frame)
        rear_right_angle = (self.vel_rear_right.heading() - tmp_right_frame)

        if degree:
            front_left_angle = math.degrees(front_left_angle)
            front_right_angle = math.degrees(front_right_angle)
            rear_left_angle = math.degrees(rear_left_angle)
            rear_right_angle = math.degrees(rear_right_angle)

        return [front_left_angle, front_right_angle, rear_left_angle, rear_right_angle]

    def get_next_speed(self):
        '''
        Get next robot speed

        :return: [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]
        '''
        scale = 3.6  # m/s to km/h
        # self.vel_front_left.mag() => cm/s
        front_left_speed = self.vel_front_left.mag() * scale / VEL_SCALE
        front_right_speed = self.vel_front_right.mag() * scale / VEL_SCALE
        rear_left_speed = self.vel_rear_left.mag() * scale / VEL_SCALE
        rear_right_speed = self.vel_rear_right.mag() * scale / VEL_SCALE
        # speed must in km/h
        return [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]


if __name__ == '__main__':
    center_x = 400
    center_y = 400
    vehicle = Vehicle(center_x, center_y, ROBOT_WIDTH / 2)

    # print(vehicle.rear_center_target.x, vehicle.rear_center_target.y, vehicle.front_center_target.x,
    #       vehicle.front_center_target.y)

    print("Target points")
    print(vehicle.front_left_target.x, vehicle.front_left_target.y, vehicle.front_right_target.x,
          vehicle.front_right_target.y)
    print(vehicle.rear_left_target.x, vehicle.rear_left_target.y, vehicle.rear_right_target.x,
          vehicle.rear_right_target.y)

    print("Real points")
    print(vehicle.front_left.x, vehicle.front_left.y, vehicle.front_right.x,
          vehicle.front_right.y)
    print(vehicle.rear_left.x, vehicle.rear_left.y, vehicle.rear_right.x,
          vehicle.rear_right.y)

    print('Validate test ==== After apply displacement')
    # vehicle.follow(224.0, 158.0)
    vehicle.follow(505.0, 375.0)
    # print(math.atan(158 / 224))
    # There is an error in coordinate at this point
    # print(vehicle.angle)

    print("Target points")
    print(vehicle.front_left_target.x, vehicle.front_left_target.y, vehicle.front_right_target.x,
          vehicle.front_right_target.y)
    print(vehicle.rear_left_target.x, vehicle.rear_left_target.y, vehicle.rear_right_target.x,
          vehicle.rear_right_target.y)

    print("Instant velocity")
    print(vehicle.vel_front_left.mag(), vehicle.vel_front_right.mag())
    print(vehicle.vel_rear_left.mag(), vehicle.vel_rear_right.mag())

    print("Instant angular")
    print(vehicle.vel_front_left.heading() * 180 / math.pi, vehicle.vel_front_right.heading() * 180 / math.pi)
    print(vehicle.vel_rear_left.heading() * 180 / math.pi, vehicle.vel_rear_right.heading() * 180 / math.pi)

    print("Instant steering")
    print(vehicle.steering_front_left * 180 / math.pi, vehicle.steering_front_right * 180 / math.pi)
    print(vehicle.steering_rear_left * 180 / math.pi, vehicle.steering_rear_right * 180 / math.pi)

    print("Real points")
    print(vehicle.front_left.x, vehicle.front_left.y, vehicle.front_right.x,
          vehicle.front_right.y)
    print(vehicle.rear_left.x, vehicle.rear_left.y, vehicle.rear_right.x,
          vehicle.rear_right.y)
