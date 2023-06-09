from DrivingInterface.drive_controller import DrivingController
from math import atan2, cos, sin, sqrt,pi

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        
        ## 코너링 관련 파라미터
        self.prev_E_cornering = 0
        self.I_cornering = 0
        
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug:
            print("=========================================================")

            # print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))
            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            # print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))

            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            # print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################
        set_brake = 0.0
        
        ## 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        if sensing_info.speed < 60: set_throttle = 1  ## 속도가 60Km/h 이하인 경우 0.9 로 설정
        # if sensing_info.speed > 80: set_throttle = 0.8  ## 최대속도를 80km/h로 설정
        else : set_throttle =  min( map_value(sensing_info.speed,0,60,0,1), 1)
        
        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25

        ## 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        ref_distance = sensing_info.distance_to_way_points[angle_num] if angle_num > 0 else 0
        reli_routeInform = route_info(sensing_info.track_forward_angles)
        
        road_range = int(sensing_info.speed / 45)
        temp = False
        is_corner(sensing_info.track_forward_angles)

        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 20:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                temp = True
                break
        
        if prepare_corner(reli_routeInform):
            

                
            ref_mid = (sensing_info.to_middle / (sensing_info.speed*1.2)) *-1
            # if is_corner(sensing_info.track_forward_angles):
            if temp:
                ready = move_to_in(sensing_info.speed, reli_routeInform, sensing_info.moving_angle, sensing_info.to_middle, half_load_width, self.I_cornering, self.prev_E_cornering)
            else:
                ready = move_to_out(sensing_info.speed, reli_routeInform, sensing_info.moving_angle, sensing_info.to_middle, half_load_width, self.I_cornering, self.prev_E_cornering)
            ref_angle -= ready
            
            # else:
            #     ready = move_to_out(sensing_info.speed, reli_routeInform, sensing_info.moving_angle, sensing_info.to_middle, half_load_width, self.I_cornering, self.prev_E_cornering)
            #     ref_angle -= ready

            # # 회전시에는 중앙정렬 불필요
            # if not is_corner(sensing_info.track_forward_angles):
            #     ref_mid = (sensing_info.to_middle / 360) * -1
        else:
            # print("코너가 아닙니다.")
            self.I_cornering = 0
            self.prev_E_cornering = 0
            ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
            ref_mid = (sensing_info.to_middle / (sensing_info.speed)) * -1.2



        ## 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.9
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.75
        if sensing_info.speed > 150: steer_factor = sensing_info.speed * 0.66
        if sensing_info.speed > 170: steer_factor = sensing_info.speed * 0.5
        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)

        ## 차선 중앙정렬 값을 추가로 고려함
        # if ref_angle == 0:
        # gen_ref_angle = map_value(abs(ref_angle),-50,50,-1,1)
        # gen_ref_angle = gen_ref_angle + 0.1 if gen_ref_angle >= 0 else gen_ref_angle - 0.1
        gen_ref_angle = map_value(abs(ref_angle),0,50,0,1) + 0.50
        middle_add = gen_ref_angle * ref_mid
        set_steering += middle_add


        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True
        emergency_brake = False

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 20)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 52:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break

        ## brake, throttle 제어
        
        if full_throttle == False:
            set_steering += 0.25
            set_brake = 0.1
            # set_brake = 0.01
            # set_steering += 0.3
            # set_brake = 0.15
            # set_throttle = 0.9
            # print(sensing_info.moving_angle)
            # set_brake = min(0.35 + map_value(abs(sensing_info.moving_angle),0,50,0,1),1)
            # print(set_brake)
        # if sensing_info.speed > 90:
        #     set_brake = 0.3
        # if sensing_info.speed > 120:
        #     set_throttle = 0.9
        #     set_brake = 0.4
        # if sensing_info.speed > 130:
        #     set_throttle = 0.8
        #     set_brake = 0.5

        ################################################################################################################

        # Moving straight forward
        car_controls.steering = max(set_steering, -0.5)
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake

        # if self.is_debug:
        #     print("[MyCar] steering:{}, throttle:{}, brake:{}" \
        #           .format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name



grid_size = 0.1
I_ax = 0
prev_E_ax = 0


def route_info(fw_angles):
    sightsee = fw_angles
    rel_YLength = 0
    rel_XLength = 0
    for fw_angle in sightsee:
        rel_YLength += sin(fw_angle * (pi/180))
        rel_XLength += cos(fw_angle * (pi/180))
    # print("차량 기준 y값 진행률", rel_YLength)
    # print("차량 기준 x값 진행률", rel_XLength)
    return (rel_XLength, rel_YLength)

def prepare_corner(road_data):
    _is_corner = False
    x, y = road_data
    # if y > 12 and x < 15 : _is_corner = True
    # elif x >= 15 : _is_corner = False
    if abs(y) > 10  : _is_corner = True
    else : _is_corner = False
    return _is_corner

def is_corner(fw_angles):
    theta1 = fw_angles[1]
    theta2 = fw_angles[2]
    R = abs(10/(2*(theta2-theta1)+0.001))
    
    print("곡률반경 R : ", R)
    return True if R <= 4.9 else False

def move_to_out(car_speed, road_data, car_yaw, car_pos, half_road_width, i_cornering, prev_e_cornering):
    x, y = road_data
    
    # y가 음수(좌측) 일 경우에는 +1, y가 양수(우측) 일 경우에는 -1
    out_is = 1 if y <0 else -1
    P = abs(map_value(x, 0, 20, 2, 1))
    I = abs(map_value(y, 0, 20, 1, 1.3))
    proximate_dist = ((car_speed /3.6)+ 4.001)
    target_pos = out_is * (half_road_width-1.5)
    next_pos = PID_controller(car_pos, target_pos * -1 , i_cornering, prev_e_cornering, factor=1, kP=P, kI=I, kD=0.0)
    
    target_angle = _required_angle(proximate_dist+x, car_pos, next_pos)
    next_angle = PID_controller(car_yaw, target_angle , i_cornering, prev_e_cornering, factor=1, kP=1, kI=1, kD=0)
    
    return next_angle

def move_to_in(car_speed, road_data, car_yaw, car_pos, half_road_width, i_cornering, prev_e_cornering):
    x, y = road_data
    # print("테스트 : ", sqrt(x**2+y**2))
    # y가 음수(좌측) 일 경우에는 +1, y가 양수(우측) 일 경우에는 -1
    out_is = 1 if y <0 else -1
    P = abs(map_value(x, 0, 20, 2, 1))
    I = abs(map_value(y, 0, 20, 1, 1.5))
    proximate_dist = ((car_speed /3.6)+ 4.001)
    # print(f"{proximate_dist}m/s")
    target_pos = out_is * (half_road_width)
    next_pos = PID_controller(car_pos, target_pos , i_cornering, prev_e_cornering, factor=1, kP=P, kI=I, kD=0.01)
    
    target_angle = _required_angle(125, car_pos, next_pos)
    next_angle = PID_controller(car_yaw, target_angle , i_cornering, prev_e_cornering, factor=1, kP=1, kI=1, kD=0.01)
    next_angle +=( atan2(y,x)* (180/pi) - 90) / 40
    return next_angle

def _required_angle(proximate_dist, car_pos, target_pos):
    target_angle = (atan2(proximate_dist, (car_pos - target_pos)) * (180/pi) - 90 )
    if abs(car_pos - target_pos) <= 0.2:
        return  target_angle + 0.5 if target_angle >= 0  else target_angle - 0.5
    return target_angle

def PID_controller(_input, target, I_ , prev_E_, factor ,kP, kI, kD):
    global I_ax, prev_E_ax
    
    dt = 0.1 # 신호가 0.1초마다 업데이트
    
    E_ax = _input + target*(factor + 0.001)
    I_ += dt * E_ax
    
    dE_ax = (E_ax - prev_E_) / dt
    # Clamp I_ax_integral to be between -1.0 and 1.0
    I_ax = min(max(I_ax, -1.0), 1.0)
    prev_E_ = E_ax

    Cont_ax = kP * E_ax + kI * I_ax + kD * dE_ax
    Cont_ax /= (factor + 0.001)
    return Cont_ax


def map_value(value, min_value, max_value, min_result, max_result):
    '''maps value (or array of values) from one range to another'''
    if (max_value - min_value) == 0: return max_result
    result = min_result + (value - min_value)/(max_value - min_value)*(max_result - min_result)
    return result

if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)



