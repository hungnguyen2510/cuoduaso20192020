import yaml
import os

config_name = 'config_map03.yaml'
config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), config_name)
class Config:
    def __init__(self):
        with open(config_path) as file:
            self.config = yaml.full_load(file)
            self.team = self.config.get('team')

            self.speed_go_straight = self.config.get('speed_go_straight')
            self.speed_go_snow = self.config.get('speed_go_snow')

            self.speed_process_tf = self.config.get('speed_process_tf')
            self.speed_process_ob = self.config.get('speed_process_ob')

            self.angle_tf_left = self.config.get('angle_tf_left')
            self.angle_tl_right = self.config.get('angle_tl_right')

            self.angle_snow_left = self.config.get('angle_snow_left')
            self.angle_snow_right = self.config.get('angle_snow_right')

            self.time_delay_tf_left = self.config.get('time_delay_tf_left')
            self.time_delay_tf_right = self.config.get('time_delay_tf_right')

            self.time_delay_ob_left = self.config.get('time_delay_ob_left')
            self.time_delay_ob_right = self.config.get('time_delay_ob_right')

            self.time_delay_snow_left = self.config.get('time_delay_snow_left')
            self.time_delay_snow_right = self.config.get('time_delay_snow_right')

            self.limit_area_ob = self.config.get('limit_area_ob')
            self.pos_ob = self.config.get('pos_ob')
            self.limit_dis_lane = self.config.get('limit_dis_lane')
            self.debug = self.config.get('debug')


    def get_limit_area_ob(self):
        return self.limit_area_ob

    def get_limit_dis_lane(self):
        return self.limit_dis_lane

    def get_pos_ob(self):
        return self.pos_ob

    def get_debug(self):
        return self.debug
        
    def get_team(self):
        return self.team

    def get_speed_go_straight(self):
        return self.speed_go_straight
    
    def get_speed_go_snow(self):
        return self.speed_go_snow
    
    def get_speed_process_tf(self):
        return self.speed_process_tf
    
    def get_speed_process_ob(self):
        return self.speed_process_ob
    
    def get_angle_tf_left(self):
        return self.angle_tf_left
    
    def get_angle_tl_right(self):
        return self.angle_tl_right
    
    def get_angle_snow_left(self):
        return self.angle_snow_left
    
    def get_angle_snow_right(self):
        return self.angle_snow_right
    
    def get_time_delay_tf_left(self):
        return self.time_delay_tf_left
    
    def get_time_delay_tf_right(self):
        return self.time_delay_tf_right
    
    def get_time_delay_ob_left(self):
        return self.time_delay_ob_left
    
    def get_time_delay_ob_right(self):
        return self.time_delay_ob_right
    
    def get_time_delay_snow_right(self):
        return self.time_delay_snow_right
    
    def get_time_delay_snow_left(self):
        return self.time_delay_snow_left
