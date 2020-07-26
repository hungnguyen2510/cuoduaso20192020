#!/usr/bin/env python
# -*- coding: utf-8 -*-
from time import time, sleep
from enum import Enum

class State(Enum):
    BRAKE = 1
    NEURAL = 2
    FORWARD = 3
    REVERSE = 4


SPEED = [10, 30, 100, 10]  # MIN, MAX, BRAKE, DEFAULT


class Lcd:

    def __init__(self):
        pass

    def set_lcd_1(self, msg):
        self.pub_lcd1.publish(msg)

    def set_lcd_2(self, msg):
        self.pub_lcd2.publish(msg)

    def set_lcd_3(self, msg):
        self.pub_lcd3.publish(msg)

    def set_lcd_4(self, msg):
        self.pub_lcd4.publish(msg)

    def clean_lcd(self, line = 0):
        self.pub_clear.publish(line)


class CarVal:
    def __init__(self):

        self.line_left_angle = 0
        self.line_mid_angle = 0
        self.line_right_angle = 0
        self.line_dotted_angle = 0

        self.line_left_curve = 0
        self.line_mid_curve = 0
        self.line_right_curve = 0
        self.line_dotted_curve = 0

        self.line_left_distance = 0
        self.line_mid_distance = 0
        self.line_right_distance = 0
        self.line_dotted_distance = 0

        self.bien_bao_cam = 0

        self.is_running = False

        self.ss_status = True
        self.imu_angle = 0
        self.parking = -1
        self.current_speed = 0
        self.update_speed_at = time()

        self.traffic_sign = ''
        self.obstacle = -999
        self.state = State.NEURAL

        self.speed_a = 20
        self.speed_b = 100


class Car:

    def reset(self):
        self.carval.imu_angle = 0
        self.carval.is_running = False
        car.set_Steer(0)

    def __init__(self):
        self.carval = CarVal()
        self.lcd = Lcd()

    def set_Steer(self, steer):
        if abs(steer) > 60:
            steer = -60 if steer < 0 else 60
        self.pub_steer.publish(steer)

    def calc_delay_speed_change(self):
        delay_time = self.carval.speed_b - (self.carval.current_speed * self.carval.speed_b) / self.carval.speed_a
        if (time() - self.carval.update_speed_at) * 1000 > delay_time:
            return True
        return False

    def set_Speed(self, speed):
        if self.carval.state == State.BRAKE:
            self.carval.state = State.NEURAL

        else:

            if speed > 0:
                if self.carval.state != State.FORWARD:
                    print('FORWARD: {}'.format(speed))
                self.carval.current_speed = speed
                self.carval.state = State.FORWARD

            if speed < 0:
                if self.carval.state != State.REVERSE:
                    print('REVERSE: {}'.format(speed))
                self.carval.current_speed = speed
                self.carval.state = State.REVERSE

        if self.carval.state == State.NEURAL:
            speed = 0
            self.carval.current_speed = speed
            print('NEURAL: {}'.format(speed))

        self.pub_speed.publish(speed)


    ######### Hàm đi tới #########
    def car_Forward(self, speed):
        if speed < SPEED[0] or self.carval.state == State.REVERSE:
            return

        self.carval.current_speed = self.car_TinhTocDo(speed)

        self.set_Speed(self.carval.current_speed)

    def car_Brake(self):
        if self.carval.state == State.FORWARD:
            self.pub_speed.publish(-SPEED[2])
            print("BRAKE FORWARD -100")
        if self.carval.state == State.REVERSE:
            self.pub_speed.publish(SPEED[2])
            print("BRAKE REVERSE 100") 
        self.carval.state = State.BRAKE

    def car_TinhTocDo(self, goal_speed):

        # ---------- Tốc độ lăn bánh nhỏ nhất ---------- #
        if self.carval.current_speed == 0:
            self.carval.update_speed_at = time()
            return SPEED[0]

        # ---------- Tăng tốc 1 đơn vị theo chu kỳ ---------- #
        if goal_speed > self.carval.current_speed:
            if self.calc_delay_speed_change() :
                self.carval.update_speed_at = time()
                self.carval.current_speed += 1

        # ---------- Giảm tốc 1 đơn vị theo chu kỳ t_down ---------- #
        if goal_speed < self.carval.current_speed:
            self.carval.current_speed = goal_speed

        return self.carval.current_speed


    def car_setLed(self, status):
        self.pub_led.publish(status)

    def car_CoLineTrai(self):
        if self.carval.line_left_angle == -999:
            return False
        return True

    def car_CoLinePhai(self):
        if self.carval.line_right_angle == 999:
            return False
        return True

    def car_Co1Trong2Line(self):
        if self.car_CoLineTrai() or self.car_CoLinePhai():
            return True
        return False

    def car_Co2Line(self):
        if self.car_CoLineTrai() and self.car_CoLinePhai():
            return True
        return False

    def car_BamTrai(self, speed, num=0, division=1, defAng=0):
        steer = (self.carval.line_left_angle + num) / division + defAng
        self.set_Steer(steer)
        self.car_Forward(speed)

    def car_BamGiua(self, speed, num=0, division=1, defAng=0):
        steer = (self.carval.line_mid_angle + num) / division + defAng
        self.set_Steer(steer)
        self.car_Forward(speed)

    def car_BamPhai(self, speed, num=0, division=1, defAng=0):
        steer = (self.carval.line_right_angle + num) / division + defAng
        self.set_Steer(steer)
        self.car_Forward(speed)

    def car_BamCongTrai(self, speed, num=0, division=1, defAng=0):
        self.set_Steer((self.carval.line_left_curve + num) / division + defAng)
        self.car_Forward(speed)

    def car_BamCongGiua(self, speed, num=0, division=1, defAng=0):
        self.set_Steer((self.carval.line_mid_curve + num) / division + defAng)
        self.car_Forward(speed)

    def car_BamCongPhai(self, speed, num=0, division=1, defAng=0):
        self.set_Steer((self.carval.line_right_curve + num) / division + defAng)
        self.car_Forward(speed)

    def car_CuaTheoGoc(self, speed, steer):
        self.set_Steer(steer)
        self.car_Forward(speed)

    def car_ChayTheoGoc(self, speed, steer):
        self.car_Forward(speed)
        self.set_Steer(steer)

    def car_resetImu(self):
        self.pub_reset_imu.publish(True)


car = Car()


def get_car():
    return car
