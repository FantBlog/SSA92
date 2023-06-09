from DrivingInterface.drive_controller import DrivingController

import math
from math import atan2, tan, pi, cos, sin, ceil, sqrt
import numpy as np
import matplotlib.pyplot as plt


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

        ## 기본 제공 파라미터
        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        
        ## 도로 정보
        Road = []
        Left_line = []
        Right_line = []

        ## 코너링 관련 파라미터
        self.prev_E_cornering = 0
        self.I_cornering = 0

        self.prev_E_positioning = 0
        self.I_positioning = 0
        
        
        ## 역주행 관련 파라미터
        self.reverse_drive = 0
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
        ################################################################################################################
        
        
        ### 함수 설정
        def plot():
            # 삼각 함수에 넣을때는 라디안으로
            A = 90
            track_forward_angles = [0,] + sensing_info.track_forward_angles
            distance_to_way_points = [sensing_info.to_middle,] + sensing_info.distance_to_way_points
            Angle_list = [0,]
            
            self.Road = []
            X_list = []
            Y_list = []

            self.Right_line = []
            Right_road_X = []
            Right_road_Y = []

            self.Left_line = []
            Left_road_X = []
            Left_road_Y = []

            for i in range(20):
                C = 180 - (track_forward_angles[i + 1] - track_forward_angles[i]) - A

                temp = distance_to_way_points[i] * math.sin(math.radians(C)) / distance_to_way_points[i + 1]
                
                temp = min(max(temp, -1), 1)
                A = math.degrees(math.asin(temp))

                car_to_point_angle = 180 - C - A

                Angle_list.append(car_to_point_angle + Angle_list[i])

                X = -math.cos(math.radians(Angle_list[i + 1])) * distance_to_way_points[i + 1]
                Y = math.sin(math.radians(Angle_list[i + 1])) * distance_to_way_points[i + 1]

                X_list.append(X)
                Y_list.append(Y)

                self.Road.append((X, Y))

                Right_road_X.append(X + math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)
                Right_road_Y.append(Y - math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)

                self.Right_line.append((X + math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit, Y - math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit))

                Left_road_X.append(X - math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)
                Left_road_Y.append(Y + math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)

                self.Left_line.append((X - math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit, Y + math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit))


            Angle_list = Angle_list[1:]

            def round3(n):
                return round(n,3)

            if self.is_debug:
                print()
                print(len(Angle_list))
                print('Angle_list : ', list(map(round3, Angle_list)))
                print('X_list : ', list(map(round3, X_list)))
                print('Y_list : ', list(map(round3, Y_list)))
            
            import matplotlib.pyplot as plt
            plt.cla()
            plt.plot(X_list, Y_list, 'b-', marker='o')
            plt.plot(Left_road_X, Left_road_Y, 'r-', marker='o')
            plt.plot(Right_road_X, Right_road_Y, 'r-', marker='o')
            # plt.show(block=False)
            # plt.pause(0.1)
        
        # # print("중앙선",self.Road)
        # # print("왼쪽경계",self.Left_line )
        # # print("오른경계",self.Right_line)
        def main_route_while_curve():
            pathX = []
            pathY = []
            pre_w = np.array([0,0])

            print("===================================================")
            for i in range(19): # y 방향이 직선구간(cos), x방향은 좌우(sin), 차량의 위치는 0,0
                xl, yl = self.Left_line[i]
                xr, yr = self.Right_line[i]
                dx, dy = xl - xr, yl - yr
                
                xl1, yl1 = self.Left_line[i+1]
                xr1, yr1 = self.Right_line[i+1]
                dx1, dy1 = xl1 - xr1, yl1 - yr1
                
                dxi = (xr1 - xr) * (sin(sensing_info.track_forward_angles[i+1] * (pi/180)) - sin(sensing_info.track_forward_angles[i] * (pi/180)))
                dyi = (yr1 - yr) * (cos(sensing_info.track_forward_angles[i+1] * (pi/180)) - cos(sensing_info.track_forward_angles[i] * (pi/180)))
                
                # print(dxi, xr1-xr, xl1-xl)
                # print(dyi, yr1-yr, yl1-yl)
                
                H11 = dx1**2 + dy1**2 
                H22 = dx**2 + dy**2
                H12 = -1*(dx1*dx + dy1*dy)
                
                B11 = 2*(dxi*dx1 + dyi*dy1)
                B12 = -2*(dxi*dx + dyi*dy)

                Hs = np.array([[H11, H12], [H12, H22]])
                Bs = np.array([B11, B12])
                
                
                try : # 역행렬이 있는경우
                    w = np.linalg.inv(Hs).dot(Bs)
                    pre_w = w
                except: # 역행렬이 없는경우
                    w = pre_w
                    
                x,y = self.Road[i+1]
                
                # print(w)
                pathX.append(max(min(xr + w[0]*(xl-xr), xr), xl))
                pathY.append(max(min(yr + w[1]*(yl-yr), yr), yl))
            print("===================================================")
            plt.plot(pathX, pathY, 'k-', marker='o')
            plt.show(block=False)
            plt.pause(0.05)
            return pathX, pathY
        
        
        def move_to_in(car_speed, road_data, car_yaw, car_pos, half_road_width):
            x, y = road_data
            # print("테스트 : ", sqrt(x**2+y**2))
            # y가 음수(좌측) 일 경우에는 +1, y가 양수(우측) 일 경우에는 -1
            out_is = 1 if y <0 else -1
            P = abs(map_value(x, 0, 20, 1.5, 1))
            I = abs(map_value(y, 0, 20, 1, 1.5))
            proximate_dist = ((car_speed /3.6)+ 4.001)
            # print(f"{proximate_dist}m/s")
            target_pos = out_is * (half_road_width-1)
            next_pos = PID_controller(car_pos, target_pos, self.I_positioning, self.prev_E_positioning, factor=1, kP=P, kI=I, kD=0)
            
            target_angle = _required_angle(125, car_pos, next_pos)
            next_angle = PID_controller(car_yaw, target_angle, self.I_cornering, self.prev_E_cornering, factor=1, kP=1, kI=1, kD=0.01)
            next_angle +=( atan2(y,x)* (180/pi) - 90) / 40
            return next_angle
        ## 함수 설정 끝
        ################################################################################################################
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            # print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            # print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            # print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            # print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################
        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25

        ## 0. 기본값 세팅
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        ref_distance = sensing_info.distance_to_way_points[angle_num] if angle_num > 0 else 0
        
        
        
        
        ###########################################################################
        # try:
        L = 4.6 # 차 길이 (m)
        # 삼각 함수에 넣을때는 라디안으로
        A = 90

        track_forward_angles = [0,] + sensing_info.track_forward_angles
        distance_to_way_points = [sensing_info.to_middle,] + sensing_info.distance_to_way_points
        Angle_list = [0,]
        X_list = []
        Y_list = []

        for i in range(20):
            C = 180 - (track_forward_angles[i + 1] - track_forward_angles[i]) - A

            temp = distance_to_way_points[i] * math.sin(math.radians(C)) / distance_to_way_points[i + 1]
            
            temp = min(max(temp, -1), 1)
            A = math.degrees(math.asin(temp))

            car_to_point_angle = 180 - C - A

            Angle_list.append(car_to_point_angle + Angle_list[i])

            X = -math.cos(math.radians(Angle_list[i + 1])) * distance_to_way_points[i + 1]
            Y = math.sin(math.radians(Angle_list[i + 1])) * distance_to_way_points[i + 1]

            X_list.append(X)
            Y_list.append(Y)

        Angle_list = Angle_list[1:]

        def round3(n):
            return round(n,3)

        if self.is_debug:
            print()
            print(len(Angle_list))
            print('Angle_list : ', list(map(round3, Angle_list)))
            print('X_list : ', list(map(round3, X_list)))
            print('Y_list : ', list(map(round3, Y_list)))
        
        target = max(int(sensing_info.speed / 20), 5)
        alpha  = Angle_list[target] - 90
        delta = math.atan2(2 * L * math.sin(math.radians(alpha)), (sensing_info.distance_to_way_points[target]))
        ###########################################################################################################
        
        
        ## 0.1 throttle 값
        if sensing_info.speed < 60: set_throttle = 1  
        else : set_throttle =  min( map_value(sensing_info.speed,0,60,0,1), 1)
        
        ## 0.2 break 값
        set_brake = 0
        
        ## 0.3. 차량의 Speed 에 따라서 핸들을 돌리는 값[steer_factor]
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.9
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.75
        if sensing_info.speed > 150: steer_factor = sensing_info.speed * 0.66
        if sensing_info.speed > 170: steer_factor = sensing_info.speed * 0.5
        
        ## 0.4 main route로 가기위한 steering angle 설정
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)
        
        
        ## 1. 도로의 형상 파악
        reli_routeInform = route_info(sensing_info.track_forward_angles)
        
        # ## 1.1 도로의 형상에 따른 수행 동작 결정
        if prepare_corner(reli_routeInform):
        #     # half_x, half_y = route_info(sensing_info.track_forward_angles[:11])
        #     # out_factor = map_value(abs(half_y), 9, 20, 0, half_load_width)
        #     # if half_y >= 0:
        #     #     out_factor *= -1
        #     # else:
        #     #     out_factor *= 1
                
        #     # ready = move_to_in(sensing_info.speed, reli_routeInform, sensing_info.moving_angle, sensing_info.to_middle, half_load_width)
        #     # # is_corner(sensing_info.track_forward_angles)
            plot()
            pathx, pathy = main_route_while_curve()
            x, y = reli_routeInform
            P = abs(map_value(x, 0, 20, 1.5, 1))
            I = abs(map_value(y, 0, 20, 1, 1.5))
            target_pos = sensing_info.to_middle + (0.2*pathx[angle_num] + 0.8 *pathx[0])/10
            print("목표",target_pos)
            print("현재",sensing_info.to_middle)
            next_pos = PID_controller(sensing_info.to_middle, target_pos , self.I_positioning, self.prev_E_positioning, factor=1, kP=P, kI=I, kD=0.0)
            print("0.1초 뒤",next_pos)
            proximate_dist = ((sensing_info.speed /3.6)+ 4.001)
            target_angle = _required_angle(proximate_dist, sensing_info.to_middle, next_pos)
            target_angle = (target_angle + sensing_info.track_forward_angles[0])/2
            next_angle = PID_controller(sensing_info.moving_angle, target_angle , self.I_cornering, self.prev_E_cornering, factor=1, kP=P, kI=I, kD=0)
            print("0.1초 뒤 각",next_angle)
        #     set_steering = min(1, sensing_info.speed/10) *  (next_angle - sensing_info.moving_angle) / (steer_factor + 0.001)
            pass
        else:
            # print("코너가 아닙니다.")
            self.I_cornering = 0
            self.prev_E_cornering = 0
            ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
            next_pos = sensing_info.to_middle

        ## 2. 장애물에 따른 장애물 극복 로직
        gen_ref_angle = map_value(abs(ref_angle),0,50,0,1) + 0.55
        ref_mid = (sensing_info.to_middle /(sensing_info.speed+0.001)) * -1.2
        ### 2.1 장애물 파악
        objects = analyze_obstacles(sensing_info.speed, sensing_info.to_middle, sensing_info.track_forward_obstacles)
        ### 2.2 전방 파악
        mapped_data = VFH_grid_map(sensing_info.speed, half_load_width, objects)
        ### 2.3 전방 데이터를 바탕으로 경로 설정
        path_recommend = path_planning(sensing_info.speed, sensing_info.moving_angle, sensing_info.to_middle, mapped_data, half_load_width, sensing_info.track_forward_obstacles)
        ### 2.4 해당 경로를 바탕으로 회피각도 설정
        
        avoidance_angle = generate_path(sensing_info.speed, sensing_info.to_middle, half_load_width, path_recommend, objects) 
        # print(avoidance_angle)
        ## 2.5 장애물 회피 각도 제공
        if avoidance_angle:
            selcted_path = 1.25 * min(1, sensing_info.speed/10) * ( map_value(abs(avoidance_angle),0,50,0,1) + 1.5)* (avoidance_angle) / (steer_factor + 0.001)
            set_steering += selcted_path
        middle_add = gen_ref_angle * ref_mid
        set_steering += middle_add
        
        # if sensing_info.speed > 100:
        #     set_brake = 0.5
        #     set_throttle = 0.7
        

        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True
        emergency_brake = False

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 20)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 30:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break

        ## brake, throttle 제어

        

        # 충돌확인
        if sensing_info.lap_progress > 0.5 and -1 < sensing_info.speed < 1 and not self.is_accident:
            self.accident_count += 1

        # 충돌인지
        if self.accident_count > 7:
            # print("사고발생")
            self.is_accident = True

        # 후진
        if self.is_accident:
            set_steering = 0.05
            set_throttle = -1
            set_brake = 0
            self.recovery_count += 1

        # 다시 진행
        if self.recovery_count > 7:
            set_throttle = 1

        # 다시 진행
        if self.recovery_count > 13:
            # print("다시시작")
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
            set_steering = 0
            set_throttle = 1

        if not sensing_info.moving_forward and not self.is_accident and (self.accident_count + self.recovery_count) < 3 and sensing_info.speed > 0:
            # print("역주행", self.reverse_drive)
            self.reverse_drive += 1
            if not self.reverse_steer:
                if sensing_info.to_middle < 0:
                    self.reverse_steer = -1
                else:
                    self.reverse_steer = 1
            
        if sensing_info.moving_forward:
            self.reverse_drive = 0
            self.reverse_steer = 0


        # 역주행
        if self.reverse_drive > 5:
            # print("역주행 수정")
            set_steering = self.reverse_steer
            set_throttle = 0.5


        # print("주행상ㅌ", sensing_info.moving_forward)
        # # print("forward", sensing_info.moving_forward)
        # print("accident_count", self.accident_count)
        # print("recovery", self.recovery_count)

        # Moving straight forward
        car_controls.steering = set_steering
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
### 주행 관련 로직

###

## 인코스 달리는 로직 == 변경예정
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

## 1. 코너 경사 정도를 알 수 있음
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
    R = abs(10/(theta2-theta1+0.001))
    
    # print("곡률반경 R : ", R)
    return True if R <= 6 else False

def move_to_out(car_speed, road_data, car_yaw, car_pos, half_road_width, i_cornering, prev_e_cornering):
    x, y = road_data
    
    # y가 음수(좌측) 일 경우에는 +1, y가 양수(우측) 일 경우에는 -1
    out_is = 1 if y <0 else -1
    P = abs(map_value(x, 0, 20, 2, 1))
    I = abs(map_value(y, 0, 20, 1, 1.3))
    proximate_dist = ((car_speed /3.6)+ 4.001)
    target_pos = out_is * (half_road_width-1)
    next_pos = PID_controller(car_pos, target_pos * -1 , i_cornering, prev_e_cornering, factor=1, kP=P, kI=I, kD=0.0)
    
    target_angle = _required_angle(proximate_dist+x, car_pos, next_pos)
    next_angle = PID_controller(car_yaw, target_angle , i_cornering, prev_e_cornering, factor=1, kP=1, kI=1, kD=0)
    
    return next_angle

## 장애물은 모든 맵 기준 2m 이며 to_middle 기준 좌우 1m입니다. 
## 1. 장애물을 분석
def analyze_obstacles(car_speed, car_pos, fw_obstacles) -> list:
    if len(fw_obstacles) == 0: 
        return []
    threshold_ttc = 4 # threshold_of time-to-colision
    
    target_obstacles = []
    for obstacles in fw_obstacles:
        obs_pos = obstacles['to_middle']
        
        if obstacles['dist'] <= 0 : continue                # 지나간 장애물일 경우 무시
        ttc = obstacles['dist']/((car_speed /3.6)+ 0.001)   # 시속에 3.6을 나누어 m/s로 차원을 맞춤
        if ttc <= threshold_ttc: target_obstacles.append(obstacles)
            
    return target_obstacles

## 2.1 VFH를 활용한 전방 파악
def VFH_grid_map(car_speed, half_road_width, obstacles) -> list:
    global grid_size
    # grid_size = 0.1 # m
    num_cells = ceil((half_road_width*2) / grid_size)
    grid_map = [0] * num_cells
    
    if not obstacles : return grid_map
    
    proximate_dist = obstacles[0]['dist'] 
    # 가장 가까운 장애물을 바탕으로 장애물을 히스토그램으로 표현
    for obstacle in obstacles:
        cell_position = int((obstacle['to_middle'] + half_road_width) / grid_size) 
        generalized = map_value(obstacle['dist'], proximate_dist, 200, 10, 0) 
        if 0 <= cell_position < num_cells:
            grid_map[cell_position] += generalized
            for i in range(1,int(1/grid_size *1.7)):  # 장애물의 좌우 폭을 고려, 그리드가 0.1m 이므로 1m씩 추가 및 여유 0.5m 추가
                if 0 <= cell_position - i < num_cells:
                    grid_map[cell_position - i] += generalized
                if 0 <= cell_position + i < num_cells:
                    grid_map[cell_position + i] += generalized
    return grid_map


## 2.2 장애물을 바탕으로 경로 분석
def path_planning(car_speed, car_yaw, car_pos, forward_map, half_road_width, obstacles) -> int:
    global grid_size
    # grid_size = 0.1 # m
    num_cells = len(forward_map)
    target_path = [0]*num_cells

    car_position = int((car_pos + half_road_width) / grid_size)
    proximate_dist = ((car_speed /3.6)+ 4.001) *4  # m/0.1s
    min_obj_dist = 200 if not obstacles else obstacles[0]['dist']
    # print(f'가까운 물체 : {min_obj_dist} 예상 이동거리 :{proximate_dist}')
    # 차량이 도로 내에 있을 경우
    if 0<= car_position < num_cells:
        
        for idx, point in enumerate(forward_map):
            if point >= 9.9 : continue # 장애물이 있는 지점은 무시 0 점
            
            
            # 자동차의 진행방향과, 현재지점과 목표지점과각도의 유사성에 대한 가중치
            target_pos = _map2pos(idx, car_pos, grid_size, half_road_width)
            target_angle = _required_angle(min_obj_dist, car_pos, target_pos)
            score_closest_angle = calculate_weight(car_yaw, target_angle, 50, max_score=20)
            weight_of_angle = calculate_weight(car_yaw, target_angle, 50, 1)

            
            # 차량 주행시 충돌 안전성에 관한 가중치
            # 현재의 위치에서 자동차의 진행 방향으로 계속 갔을 경우 충돌이 있는지 계산해야함
            weight_of_obstacle = calculate_weight(forward_map[idx], 0, 10,1) 
            score_closest_point = calculate_weight(car_position, idx, num_cells, 80) 
            
            for i in range(1,int(1/grid_size *1.7)):  # 차량의 폭을 고려, 그리드가 0.1m 이므로 1m씩 추가 및 여유 0.5m 추가
            # 해당 지점에 히스토그램의 값이10 이상인 장애물이 있을경우, 가중치를 최소화
                if (0 <= idx - i < num_cells and 0< forward_map[idx - i] >= 9.99):
                    weight_of_obstacle = 0.3
                    break
                if (0 <= idx + i < num_cells and 0< forward_map[idx + i] >= 9.99):
                    weight_of_obstacle = 0.3
                    break
            
            if (proximate_dist) > sqrt(((car_position - idx)* 0.1)**2 + min_obj_dist**2):
                weight_of_angle = 0.1
            
            
            ## 직선주행 중일 경우, 장애물에 대한 가중치를 높게봄
            if abs(car_yaw) < 5:  
                weight_of_obstacle *= 1.5
    
            
            # print((car_position - idx)* 0.1, "m")
            # if (car_position - idx)* 0.1
            # 가중치 매핑
            target_path[idx] = round(score_closest_point * weight_of_obstacle + weight_of_angle * score_closest_angle, 2)
        
        # 가장 가중치가 높은 지점을 추출
        target_way_points = get_max_pointed_path(target_path)
        ## 차량의 위치 기준으로 경로 세분화 
        if car_position <= half_road_width : # 차량이 좌측이 위치할 경우
            target_point = max(target_way_points) # 우측 경로로 주행 

        if car_position > half_road_width : # 차량이 우측에 위치할 경우
            target_point = min(target_way_points) # 좌측 경로로 주행
        # print(forward_map)
        # print(target_path)
        return target_point
    else:
        # print("트랙을 벗어났습니다.")
        return 0 if car_position < 0 else num_cells - 1

def get_max_pointed_path(lst):
    max_value = max(lst)
    max_indexes = [i for i, value in enumerate(lst) if value == max_value]
    return max_indexes

def calculate_weight(car_inform, target_inform, total, max_score=100):
    diff = abs(car_inform - target_inform)
    weight = round(max_score * (1 - diff/total), 1) 

    return max(0, weight)

def generate_path(car_speed, car_pos, half_road_width, recommended_path, obstacles):
    global grid_size
    target_pos = _map2pos(recommended_path, car_pos, grid_size, half_road_width)
    
    if not obstacles : return 0
    # to_obstacle = obstacles[0]['dist'] if obstacles[0]['dist'] > 0 else 0
    # new_point = _required_angle(to_obstacle, car_pos, target_pos)
    # # r = to_obstacle * tan(new_point * (pi/180))*0.5
    
    proximate_dist = ((car_speed /3.6) + 4.001) # 자동차 주행 속도를 바탕으로 angle값 결정    
    required_angle = _required_angle(proximate_dist, car_pos, (target_pos))
    return required_angle 

def _map2pos(recommended_path, car_pos, grid_size, half_road_width):
    target_pos = ((recommended_path+1 ) * grid_size - half_road_width) # (idx +1) * 그리드 - width = to_middle inform
    return target_pos + 0.1 if car_pos <= target_pos else target_pos - 0.1 # 여유값 제공

def _required_angle(proximate_dist, car_pos, target_pos):
    target_angle = (atan2(proximate_dist, (car_pos - target_pos)) * (180/pi) - 90 )
    if abs(car_pos - target_pos) <= 0.2:
        return  target_angle + 0.5 if target_angle >= 0  else target_angle - 0.5
    return target_angle
    

def PID_controller(_input, target, I_ , prev_E_, factor ,kP, kI, kD):
    
    dt = 0.1 # 신호가 0.1초마다 업데이트
    
    E_ax = target*(factor + 0.001) - _input
    I_ += dt * E_ax
    
    dE_ax = (E_ax - prev_E_) / dt
    # Clamp I_ax_integral to be between -1.0 and 1.0
    prev_E_ = E_ax

    Cont_ax = kP * E_ax + kI * I_ + kD * dE_ax
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



