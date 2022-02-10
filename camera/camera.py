# 2020-08-07 15:36
# Kim Gyu Tae
# Smart Trailer

import cv2
import numpy as np
import serial
import threading
from playsound import playsound
import time
import math
import queue

# Set PID gains
front_Kp = 0.4   # P는 현재 상태에서의 오차값의 크기에 비례
front_Ki = 0.01  # I는 정상 상태 오차를 감소 (P의 보조)
front_Kd = 0.5   # D는 급격한 변화에 제동
rear_Kp = 0.4    # P는 현재 상태에서의 오차값의 크기에 비례
rear_Ki = 0.01   # I는 정상 상태 오차를 감소 (P의 보조)
rear_Kd = 0.5    # D는 급격한 변화에 제동

front_i_limit = 10  # 전륜 D의 한계치 설정
rear_i_limit = 10   # 후륜 D의 한계치 설정

# 시리얼 통신
PORT = 'COM6'  # 여기 포트 변경 바람
BaudRate = 115200
ser = serial.Serial(PORT, BaudRate)
Connect_Wait = 0  # 연결 대기 횟수
receive_check = False  # 아두이노로부터 수신 여부 확인

# -----------------------------------------------------------------------------------------

front_video = cv2.VideoCapture(2)  # 전면 연결된 카메라
rear_video = cv2.VideoCapture(0)   # 후면 연결된 카메라

# front_video = cv2.VideoCapture("sample_3.mp4")  # 전면 샘플 영상
# rear_video = cv2.VideoCapture("sample_4.mp4")   # 후면 샘플 영상

# 카메라 화면 크기
width = 640
height = 480

s_height = height * 2  # 전체 화면 높이
state_video = np.zeros((height * 2, width, 3), np.uint8)  # 상태창 화면 초기화
top_car_pic = cv2.imread('top_car.png')  # 트랙터 사진
all_car_pic = cv2.imread('all_top_view.png')  # 트랙터, 트레일러 사진
all_car_pic = cv2.resize(all_car_pic, (80, 230))  # 트랙터, 트레일러 사진 크기 조정

# -----------------------------------------------------------------------------------------

# 인식 범위(Range of Interest) - (왼쪽 위, 왼쪽 아래, 오른쪽 아래, 오른쪽 위)
front_vertices = np.array([[(240, height - 170), (width / 2 - 40, height - 130), (width / 2 + 50, height - 130), (width - 240, height - 170)]], dtype=np.int32)
rear_vertices = np.array([[(240, height - 170), (width / 2 - 40, height - 130), (width / 2 + 50, height - 130), (width - 240, height - 170)]], dtype=np.int32)
reverse_vertices = np.array([[(250, 250), (250, 350), (width - 250, 350), (width - 250, 250)]], dtype=np.int32)

# Perspective Transform 범위
src = np.array([[0, height / 2], [width, height / 2], [0, 0], [width, 0]], dtype=np.float32)  # 원래 이미지에서 가져올 범위
dst = np.array([[0, 0], [width, 0], [width - 200, height + 150], [200, height + 150]], dtype=np.float32)  # 가져온 이미지 펼칠 범위
cut = np.array([[(0, height), (0, 240), (width, 240), (width, height)]], dtype=np.int32)  # 원래 이미지에 합성하기 위한 여백 공간 범위

# -----------------------------------------------------------------------------------------

road_width = 90  # 도로 폭

x_now = int(width / 2)   # 전면 차선 중앙 위치
x_now2 = int(width / 2)  # 후면 차선 중앙 위치

left_lane = [0, 0, 0, 0]    # 전면 좌 차선 인식 좌표
right_lane = [0, 0, 0, 0]   # 전면 우 차선 인식 좌표
left_lane2 = [0, 0, 0, 0]   # 후면 좌 차선 인식 좌표
right_lane2 = [0, 0, 0, 0]  # 후면 우 차선 인식 좌표

# 전면 차선 좌표
x_1_l = int(width / 2)
x_2_l = int(width / 2)
x_1_r = int(width / 2)
x_2_r = int(width / 2)

# 후면 차선 좌표
x_1_l2 = int(width / 2)
x_2_l2 = int(width / 2)
x_1_r2 = int(width / 2)
x_2_r2 = int(width / 2)

front_left_out = 0   # 이탈 감지 카운트
front_right_out = 0  # 이탈 감지 카운트
rear_left_out = 0    # 이탈 감지 카운트
rear_right_out = 0   # 이탈 감지 카운트
min_out = 30         # 최소 차선 이탈 카운트

# -----------------------------------------------------------------------------------------

# initiation Value
front_p = 0.0  # 자율주행시 전면 P
front_i = 0.0  # 자율주행시 전면 I
front_d = 0.0  # 자율주행시 전면 D
front_cte_previous = 0.0  # 가장 최근의 CTE

rear_p = 0.0   # 자율주행시 후면 P
rear_i = 0.0   # 자율주행시 후면 I
rear_d = 0.0   # 자율주행시 후면 D
rear_cte_previous = 0.0  # 가장 최근의 CTE

throttle_count = 0  # 전진 여부 카운터

# -----------------------------------------------------------------------------------------

car_mode = 0  # 자동차 모드 0:P, 1:D, 2:R, 3:S, 4:Show Road

front_steer = 90  # 다음 전륜 조향각 (Python)
rear_steer = 90   # 다음 후륜 조향각 (Python)
speed = 0         # 다음 속도(0~255)(Python)
direction = 0     # 다음 구동 방향   (Python)

real_front_steer = 90  # 현재 전륜 조향각 (Arduino)
real_rear_steer = 90   # 현재 후륜 조향각 (Arduino)
real_speed = 0         # 현재 속도(0~255)(Arduino)
real_direction = 0     # 현재 구동 방향(0:전진, 1:후진) (Arduino)
real_middle = 180      # 현재 굴절각(0~359) (Arduino)
real_distances = [250, 250, 250, 250, 250, 250]  # 현재 초음파 센서값 20~250cm 까지만 인식 (Arduino)

manual_steer = 0  # 수동 조작 모드, 0:평상시, 1:전륜 좌회전, 2:전륜 우회전, 3:후륜 좌회전, 4:후륜 우회전, 5:전륜 좌/후륜 우회전, 6:전륜 우/후륜 좌회전
manual_speed = 0  # 수동 조작 속도
manual_front_steer_count = 0  # 전륜 안전 카운터(수동 조작시 리미티드로 인해 과회전 방지)
manual_rear_steer_count = 0   # 후륜 안전 카운터(수동 조작시 리미티드로 인해 과회전 방지)

middle_sum = 0  # 굴절각 평균 총합
middle_queue = queue.Queue()  # 굴절각 Queue
parking_chance = 0  # 주차 공간 탐색 순서 카운터
reverse_chance = 0  # 후진 주차 순서 카운터
reverse_time = 0    # 주차 최대 굴절시 시간
dead_line = 75      # 후진 주차시 주차 완료 거리
dead_line_count = 0  # 카메라 차선 인식시 평행 인식 카운터
max_dead_line_count = 5  # 카메라 차선 인식시 평행 카운터 역치

max_front_angle = 90 + 30  # 전륜 최대 조향 (우회전)
min_front_angle = 90 - 30  # 전륜 최소 조향 (좌회전)
max_rear_angle = 90 + 30  # 후륜 최대 조향 (우회전)
min_rear_angle = 90 - 30  # 후륜 최소 조향 (좌회전)
max_speed = 240  # 자율주행 최고 속도
min_speed = 175  # 자율주행 최소 속도
up_speed = 10    # 증가 속도 폭
down_speed = 20  # 김서 속도 폭
parking_speed = 160  # 자율주차 속도
search_speed = 140   # 주차 공간 탐색 속

# 초음파 거리 - 거리에 따른 소리와 표시를 위한 범위 설정(단위 CM)
dis_1 = 110
dis_2 = 90
dis_3 = 70
dis_4 = 50

# -----------------------------------------------------------------------------------------

mode_location = (int(width / 2) - 250, 100)  # P D R S 출력 위치
speed_location = (int(width / 2) - 25, 100)  # 속도 출력 위치
autopilot_location = (int(width / 2) + 170, 100)  # AUTO 출력 위치

font = cv2.FONT_ITALIC  # normal size sans-serif font
fontScale = 1

sound_num = 0  # 소리 출력 번호

pid_mode = 0  # 수동 PID 조작 모드 (숫자 1,2,3,4,5,6 키)
pid_ud = 0    # 수동 pid 조작 Up or Down (-(_),=(+)키)


# -----------------------------------------------------------------------------------------


# Python -> Arduino 시리얼 통신
def send_data():
    global manual_steer
    if manual_steer == 0: # Front_Steering / Rear_Steering / Speed / Direction 순의 데이터 정리 후 전송
        try:
            if front_steer < 100:
                op_d = "C0" + str(front_steer)
            else:
                op_d = "C" + str(front_steer)

            if rear_steer < 100:
                op_d = op_d + "0" + str(rear_steer)
            else:
                op_d = op_d + str(rear_steer)

            if speed < 10:
                op_d = op_d + "00" + str(speed) + str(direction)
            elif speed < 100:
                op_d = op_d + "0" + str(speed) + str(direction)
            else:
                op_d = op_d + str(speed) + str(direction)
            ser.write(op_d.encode('utf-8'))
        except ValueError:
            print("Fail send data")
    elif manual_steer == 1 and manual_front_steer_count > -30:  # 수동 전륜 좌회전
        try:
            op_f_l = "D"
            ser.write(op_f_l.encode('utf-8'))
            manual_steer = 0
        except ValueError:
            print("Fail send data")
            manual_steer = 0
    elif manual_steer == 2 and manual_front_steer_count < 30:  # 수동 전륜 우회전
        try:
            op_f_r = "E"
            ser.write(op_f_r.encode('utf-8'))
            manual_steer = 0
        except ValueError:
            print("Fail send data")
            manual_steer = 0
    elif manual_steer == 3 and manual_rear_steer_count > -30:  # 수동 후륜 좌회전
        try:
            op_r_l = "F"
            ser.write(op_r_l.encode('utf-8'))
            manual_steer = 0
        except ValueError:
            print("Fail send data")
            manual_steer = 0
    elif manual_steer == 4 and manual_rear_steer_count < 30:  # 수동 후륜 우회전
        try:
            op_r_r = "G"
            ser.write(op_r_r.encode('utf-8'))
            manual_steer = 0
        except ValueError:
            print("Fail send data")
            manual_steer = 0
    elif manual_steer == 5 and manual_front_steer_count > -30 and manual_rear_steer_count < 30:  # 수동 전륜 좌/후륜 우회전
        try:
            op_m_l = "H"
            ser.write(op_m_l.encode('utf-8'))
            manual_steer = 0
        except ValueError:
            print("Fail send data")
            manual_steer = 0
    elif manual_steer == 6 and manual_front_steer_count < 30 and manual_rear_steer_count > -30:  # 수동 전륜 우/후륜 좌회전
        try:
            op_m_r = "I"
            ser.write(op_m_r.encode('utf-8'))
            manual_steer = 0
        except ValueError:
            print("Fail send data")
            manual_steer = 0


# Arduino -> Python
# 전륜 현재 회전각, 후륜 현재 회전각, 현재 굴절각, 현재 속도, 현재 방향(전/후진), 초음파 6개
def serial_receive():
    global real_front_steer
    global real_front_steer
    global real_rear_steer
    global real_speed
    global real_direction
    global real_middle
    global real_distances
    global receive_check
    global car_mode
    global middle_sum
    global middle_queue

    while True:
        try:
            if ser.readable():
                data_ = ser.readline()
                data_ = data_.decode()
                if len(data_) == 33:  # 입력 길이를 기본으로 정리
                    receive_check = True
                    real_front_steer = int(data_[0:3])
                    real_rear_steer = int(data_[3:6])
                    real_middle_temp = int(data_[6:9])
                    real_speed = int(data_[9:12])
                    real_direction = int(data_[12:13])
                    for n in range(0, 6):  # 5씩 변경하도록 하여 튀는 값 제어
                        real_distances_temp = int(data_[13 + 3 * n:16 + 3 * n])
                        if real_distances[n] < real_distances_temp:
                            real_distances[n] += 5
                        elif real_distances[n] > real_distances_temp:
                            real_distances[n] -= 5
                    middle_queue.put(real_middle_temp)  # 굴절각 Queue 추가
                    middle_sum += real_middle_temp  # 굴절각 총 합(평균을 위한 값)
                    if middle_queue.qsize() > 50:  # 50개의 queue 사이즈
                        middle_sum -= middle_queue.get()  # 오래된 데이터 삭제
                        real_middle = int(middle_sum / middle_queue.qsize())  # 현재 50개의 평균

                        #  굴절각 90~270 범위 지정
                        if real_middle < 90:
                            real_middle = 90
                        elif real_middle > 270:
                            real_middle = 270

                    detail_data = "F:" + data_[0:3] + " R:" + data_[3:6] + " M:" + data_[6:9] + " S:" + data_[9:12] + " D:" + data_[12:13] + " E_F:" + data_[13:16] + " E_R:" + data_[16:19] + " E_A_L:" + data_[19:22] + " E_A_R:" + data_[22:25] + " E_B_L:" + data_[25:28] + " E_B_R:" + data_[28:31]
                    # print(detail_data)
                ser.flushInput()  # 가장 최근 데이터 수집을 위한 플래쉬 삭제
        except ValueError:
            print("Fail receive data")


# -----------------------------------------------------------------------------------------

def car_control():
    global car_mode
    global front_steer
    global rear_steer
    global speed
    global direction

    global sound_num
    global manual_steer
    global parking_chance
    global reverse_chance
    global dead_line_count
    global reverse_time
    global real_middle
    global middle_sum
    global middle_sum

    # 정지 상태(P) - Speed Down
    if car_mode == 0:
        if speed > 0 and speed - down_speed > 0:
            speed -= down_speed
        else:
            speed = 0

    # 자율 주행(D) - PID 제어, 전진, Speed Up
    elif car_mode == 1:
        pid_control()
        direction = 0
        if speed < min_speed:
            speed += up_speed
        if front_left_out > min_out and front_right_out > min_out:  # 전방 차선 이탈시 정지(P)
            car_mode = 0

    # 후진 주차 (전륜 87 / 후륜 70 -> 후진 -> 굴절각(220) 만족 -> 정지 -> 전륜 120 / 후륜 73 -> 후진 -> 후방 초음파 75cm 감지 -> 정지 -> 주차 종료)
    elif car_mode == 2:
        direction = 1  # 후진
        if sound_num != 9 and sound_num != 10 and real_distances[0] > dis_1 and real_distances[1] > dis_1 and real_distances[2] > dis_1 and real_distances[3] > dis_1 and real_distances[4] > dis_1 and real_distances[5] > dis_1:
            sound_num = 4

        if reverse_chance == 0:  # 전륜, 후륜 조향각 변경
            front_steer = 87
            rear_steer = 70
            if real_front_steer == 87 and real_rear_steer == 70:  # 변경 완료 확인
                reverse_chance = 1
        elif reverse_chance == 1:  # 후진 -> 굴절각 만족
            if speed < 190:
                speed += up_speed
            if real_middle > 220:
                reverse_chance = 2
        elif reverse_chance == 2:  # 정지
            if speed > 0 and speed - down_speed > 0:
                speed -= down_speed
            else:
                speed = 0
                reverse_chance = 3
        elif reverse_chance == 3:  # 전륜, 후륜 조향각 변경
            front_steer = 120
            rear_steer = 73
            if real_front_steer == 120 and real_rear_steer == 73:  # 변경 완료 확인
                reverse_chance = 4
        elif reverse_chance == 4:  # 후진 -> 초음파 인식 -> 후진 주차 종료
            if real_distances[1] < dead_line:
                if speed > 0 and speed - down_speed > 0:
                    speed -= down_speed
                else:
                    sound_num = 11
                    speed = 0
                    steering_reset()
                    car_mode = 0
            elif speed < 230:
                speed += up_speed

        print("Reverse", reverse_chance)

    # 주차 탐색 (속도 감속/굴절각 초기화 -> 전진 -> 1박스 접촉 -> 1박스 손실 -> 2박스 접촉 -> 정지 -> 굴절각 초기화 -> 후진 주차 시작)
    elif car_mode == 3:
        direction = 0

        if parking_chance == 0 and speed > 0 and speed - down_speed > 0:
            speed -= down_speed
        elif parking_chance == 0:
            speed = 0
            real_middle = 180
            middle_sum = 0
            with middle_queue.mutex:
                middle_queue.queue.clear()
            parking_chance = 1

        if parking_chance != 3 and front_steer == 90 and rear_steer == 90 and speed < search_speed:
            speed += up_speed

        if parking_chance == 1 and real_distances[5] < 150:  # 1 Box Contact
            parking_chance = 2
        elif parking_chance == 2 and real_distances[5] > 150:  # 1 Box DisContact
            parking_chance = 3
        elif parking_chance == 3 and real_distances[5] < 150:  # 2 Box Contact
            if speed > 0 and speed - down_speed > 0:
                speed -= down_speed
            else:
                sound_num = 9
                speed = 0
                real_middle = 180
                middle_sum = 0
                with middle_queue.mutex:
                    middle_queue.queue.clear()
                car_mode = 2
                reverse_chance = 0
                dead_line_count = 0

        print("Search", parking_chance)

    elif car_mode == 11 or car_mode == 22:  # 수동 전진/후진
        if speed < manual_speed:
            speed += up_speed
        elif speed > manual_speed:
            speed -= down_speed

    # 최대 조향 통제
    if front_steer < min_front_angle:
        front_steer = min_front_angle
    elif front_steer > max_front_angle:
        front_steer = max_front_angle
    if rear_steer < min_rear_angle:
        rear_steer = min_rear_angle
    elif rear_steer > max_rear_angle:
        rear_steer = max_rear_angle

    if car_mode != 2 and (real_distances[0] < dis_4 or real_distances[1] < dis_4):  # 물체 감지 긴급 정지
        sound_num = 15
        car_mode = 0
        pid_reset()
        steering_reset()


# 조향 초기화
def steering_reset():
    global front_steer
    global rear_steer
    global manual_front_steer_count
    global manual_rear_steer_count

    front_steer = 90
    rear_steer = 90
    manual_front_steer_count = 0
    manual_rear_steer_count = 0


# -----------------------------------------------------------------------------------------

# PID 초기화
def pid_reset():
    global front_p
    global front_i
    global front_d
    global front_cte_previous
    global rear_p
    global rear_i
    global rear_d
    global rear_cte_previous
    global throttle_count

    front_p = 0.0
    front_i = 0.0
    front_d = 0.0
    front_cte_previous = 0
    rear_p = 0.0
    rear_i = 0.0
    rear_d = 0.0
    rear_cte_previous = 0.0
    throttle_count = 0


# PID 제어
def pid_control():
    global front_p
    global front_i
    global front_d
    global front_cte_previous
    global rear_p
    global rear_i
    global rear_d
    global rear_cte_previous

    global front_steer
    global rear_steer

    global throttle_count
    global speed

    # 차의 중앙과 차선 중앙 차이
    front_cte = x_now - width / 2
    rear_cte = x_now2 - width / 2

    # PID 제어 계산
    if front_cte_previous == 0.0:
        front_cte_previous = front_cte
    front_p = front_Kp * front_cte
    front_i += front_Ki * front_cte
    front_d = -front_Kd * (front_cte - front_cte_previous)
    front_cte_previous = front_cte

    if rear_cte_previous == 0.0:
        rear_cte_previous = rear_cte
    rear_p = rear_Kp * rear_cte
    rear_i += rear_Ki * rear_cte
    rear_d = -rear_Kd * (rear_cte - rear_cte_previous)
    rear_cte_previous = rear_cte

    # I 최대 범위 지정 - 방향 전환시 느린 반응 제거
    if front_i > front_i_limit:
        front_i = front_i_limit
    elif front_i < -front_i_limit:
        front_i = -front_i_limit

    if rear_i > rear_i_limit:
        rear_i = rear_i_limit
    elif rear_i < -rear_i_limit:
        rear_i = -rear_i_limit

    # 다음 전륜 조향과 PID 결과를 비교하여 1씩 변경
    if front_steer > int(90 + (front_p + front_i + front_d)):
        front_steer -= 1
    elif front_steer < int(90 + (front_p + front_i + front_d)):
        front_steer += 1

    # 다음 후륜 조향과 PID 결과를 비교하여 1씩 변경
    if rear_steer > int(90 + (rear_p + rear_i + rear_d)):
        rear_steer -= 1
    elif rear_steer < int(90 + (rear_p + rear_i + rear_d)):
        rear_steer += 1

    # 굴절각 급격할 시 속도 상승
    if (real_front_steer <= 65 or real_front_steer >= 115) and speed < max_speed:
        speed += up_speed
    elif speed > min_speed:
        speed -= down_speed


# 수동 PID 변경
def pid_change():
    global front_Kp
    global front_Ki
    global front_Kd
    global rear_Kp
    global rear_Ki
    global rear_Kd
    global pid_ud

    if pid_mode == 1:
        if pid_ud == 1:
            front_Kp += 0.01
            front_Kp = round(front_Kp, 2)
            pid_ud = 0
        elif pid_ud == 2:
            front_Kp -= 0.01
            front_Kp = round(front_Kp, 2)
            pid_ud = 0
    elif pid_mode == 2:
        if pid_ud == 1:
            front_Ki += 0.01
            front_Ki = round(front_Ki, 2)
            pid_ud = 0
        elif pid_ud == 2:
            front_Ki -= 0.01
            front_Ki = round(front_Ki, 2)
            pid_ud = 0
    elif pid_mode == 3:
        if pid_ud == 1:
            front_Kd += 0.01
            front_Kd = round(front_Kd, 2)
            pid_ud = 0
        elif pid_ud == 2:
            front_Kd -= 0.01
            front_Kd = round(front_Kd, 2)
            pid_ud = 0

    elif pid_mode == 4:
        if pid_ud == 1:
            rear_Kp += 0.01
            rear_Kp = round(rear_Kp, 2)
            pid_ud = 0
        elif pid_ud == 2:
            rear_Kp -= 0.01
            rear_Kp = round(rear_Kp, 2)
            pid_ud = 0
    elif pid_mode == 5:
        if pid_ud == 1:
            rear_Ki += 0.01
            rear_Ki = round(rear_Ki, 2)
            pid_ud = 0
        elif pid_ud == 2:
            rear_Ki -= 0.01
            rear_Ki = round(rear_Ki, 2)
            pid_ud = 0
    elif pid_mode == 6:
        if pid_ud == 1:
            rear_Kd += 0.01
            rear_Kd = round(rear_Kd, 2)
            pid_ud = 0
        elif pid_ud == 2:
            rear_Kd -= 0.01
            rear_Kd = round(rear_Kd, 2)
            pid_ud = 0


# -----------------------------------------------------------------------------------------

# 그레이화한 영상을 더 어둡게 처리
def adjust_gamma(image, gamma=1.0):
    gamma = 1.0 / gamma
    table = np.array([((k / 255.0) ** gamma) * 255
                      for k in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


# 관심영역 설정
def roi(now_frame, set_vertices):
    mask = np.zeros_like(now_frame)
    cv2.fillPoly(mask, set_vertices, 255)
    masked = cv2.bitwise_and(now_frame, mask)
    return masked


# 인식된 차선 계산
def draw_line():
    global x_1_l
    global x_2_l
    global x_1_r
    global x_2_r

    if car_mode == 1 or car_mode == 4:
        if left_lane[0] != 0:
            if left_lane[0] != left_lane[2]:
                a = left_lane[1] - int(((left_lane[3] - left_lane[1]) / (left_lane[2] - left_lane[0])) * left_lane[0])
                p_x1_l = int((height - 120 - a) / ((left_lane[3] - left_lane[1]) / (left_lane[2] - left_lane[0])))
                p_x2_l = int((height / 2 - a) / ((left_lane[3] - left_lane[1]) / (left_lane[2] - left_lane[0])))
            else:
                p_x1_l = left_lane[0]
                p_x2_l = left_lane[2]

            # 급격한 차선 움직임 제거
            if p_x1_l > x_1_l:
                x_1_l += 2
            elif p_x1_l < x_1_l:
                x_1_l -= 2

            if p_x2_l > x_2_l:
                x_2_l += 2
            elif p_x2_l < x_2_l:
                x_2_l -= 2

        if right_lane[0] != width:
            if right_lane[0] != right_lane[2]:
                b = right_lane[1] - int(((right_lane[3] - right_lane[1]) / (right_lane[2] - right_lane[0])) * right_lane[0])
                p_x1_r = int((height - 120 - b) / ((right_lane[3] - right_lane[1]) / (right_lane[2] - right_lane[0])))
                p_x2_r = int((height / 2 - b) / ((right_lane[3] - right_lane[1]) / (right_lane[2] - right_lane[0])))
            else:
                p_x1_r = right_lane[0]
                p_x2_r = right_lane[2]

            if p_x1_r > x_1_r:
                x_1_r += 2
            elif p_x1_r < x_1_r:
                x_1_r -= 2

            if p_x2_r > x_2_r:
                x_2_r += 2
            elif p_x2_r < x_2_r:
                x_2_r -= 2

        # 한쪽 차선 소실시 반대쪽 차선 생성
        if front_left_out > 15:
            x_1_l = x_1_r - road_width
            x_2_l = x_2_r - road_width
        elif front_right_out > 15:
            x_1_r = x_1_l + road_width
            x_2_r = x_2_l + road_width
    else:  # 차선 사라지기
        if x_1_l < int(width / 2):
            x_1_l += 2
        elif x_1_l > int(width / 2):
            x_1_l -= 2
        if x_1_r < int(width / 2):
            x_1_r += 2
        elif x_1_r > int(width / 2):
            x_1_r -= 2
        if x_2_l < int(width / 2):
            x_2_l += 2
        elif x_2_l > int(width / 2):
            x_2_l -= 2
        if x_2_r < int(width / 2):
            x_2_r += 2
        elif x_2_r > int(width / 2):
            x_2_r -= 2


def draw_line2():
    global x_1_l2
    global x_2_l2
    global x_1_r2
    global x_2_r2

    if car_mode == 1 or car_mode == 4:
        if left_lane2[0] != 0:
            if left_lane2[0] != left_lane2[2]:
                a = left_lane2[1] - int(((left_lane2[3] - left_lane2[1]) / (left_lane2[2] - left_lane2[0])) * left_lane2[0])
                p_x1_l = int((height - 120 - a) / ((left_lane2[3] - left_lane2[1]) / (left_lane2[2] - left_lane2[0])))
                p_x2_l = int((height / 2 - a) / ((left_lane2[3] - left_lane2[1]) / (left_lane2[2] - left_lane2[0])))
            else:
                p_x1_l = left_lane2[0]
                p_x2_l = left_lane2[2]

            # 급격한 차선 움직임 제거
            if p_x1_l > x_1_l2:
                x_1_l2 += 2
            elif p_x1_l < x_1_l2:
                x_1_l2 -= 2

            # 급격한 차선 움직임 제거
            if p_x2_l > x_2_l2:
                x_2_l2 += 2
            elif p_x2_l < x_2_l2:
                x_2_l2 -= 2

        if right_lane2[0] != width:
            if right_lane2[0] != right_lane2[2]:
                b = right_lane2[1] - int(
                    ((right_lane2[3] - right_lane2[1]) / (right_lane2[2] - right_lane2[0])) * right_lane2[0])
                p_x1_r = int((height - 120 - b) / ((right_lane2[3] - right_lane2[1]) / (right_lane2[2] - right_lane2[0])))
                p_x2_r = int((height / 2 - b) / ((right_lane2[3] - right_lane2[1]) / (right_lane2[2] - right_lane2[0])))
            else:
                p_x1_r = right_lane2[0]
                p_x2_r = right_lane2[2]

            # 급격한 차선 움직임 제거
            if p_x1_r > x_1_r2:
                x_1_r2 += 2
            elif p_x1_r < x_1_r2:
                x_1_r2 -= 2

            # 급격한 차선 움직임 제거
            if p_x2_r > x_2_r2:
                x_2_r2 += 2
            elif p_x2_r < x_2_r2:
                x_2_r2 -= 2
        # 한쪽 차선 소실시 반대쪽 차선 생성
        if rear_left_out > 15:
            x_1_r2 - road_width
            x_2_l2 = x_2_r2 - road_width
        elif rear_right_out > 15:
            x_1_r2 = x_1_l2 + road_width
            x_2_r2 = x_2_l2 + road_width
    else:  # 차선 사라지기
        if x_1_l2 < int(width / 2):
            x_1_l2 += 2
        elif x_1_l2 > int(width / 2):
            x_1_l2 -= 2
        if x_1_r2 < int(width / 2):
            x_1_r2 += 2
        elif x_1_r2 > int(width / 2):
            x_1_r2 -= 2
        if x_2_l2 < int(width / 2):
            x_2_l2 += 2
        elif x_2_l2 > int(width / 2):
            x_2_l2 -= 2
        if x_2_r2 < int(width / 2):
            x_2_r2 += 2
        elif x_2_r2 > int(width / 2):
            x_2_r2 -= 2


# 인식된 선 처리
# 왼쪽 차선은 전 차선 중앙을 기준으로 왼쪽에서 가장 가까운 선
# 오른쪽 차선은 전 차선 중앙을 기준으로 오른쪽에서 가장 가까운 선
# 차선 중앙 처리
def line_process():
    global front_left_out
    global front_right_out
    global rear_left_out
    global rear_right_out
    global x_now
    global x_now2
    global dead_line_count

    if front_lines is not None:
        left = [0, 0, 0, 0]
        right = [width, 0, width, 0]
        for line in front_lines:
            x1, y1, x2, y2 = line[0]
            if x1 <= x_now:
                if left[0] < x1 and abs(y1 - y2) > 10:
                    left[0] = x1
                    left[1] = y1
                    left[2] = x2
                    left[3] = y2
                    front_left_out = 0
                    front_right_out += 1
            else:
                if x1 < right[0] and abs(y1 - y2) > 10:
                    right[0] = x1
                    right[1] = y1
                    right[2] = x2
                    right[3] = y2
                    front_right_out = 0
                    front_left_out += 1
            # cv2.line(front_warped_frame, (x1, y1), (x2, y2), (51, 104, 255), 2)  # 인식 확인

        for i in range(0, 4):
            left_lane[i] = left[i]
            right_lane[i] = right[i]
    else:
        front_right_out += 1
        front_left_out += 1

    draw_line()

    x_now = int(((x_1_l + x_2_l) / 2 + (x_1_r + x_2_r) / 2) / 2)

    # -----------------------------------------------------------------------------------------

    if rear_lines is not None:
        left = [0, 0, 0, 0]
        right = [width, 0, width, 0]
        for line in rear_lines:
            x1, y1, x2, y2 = line[0]
            if x1 <= x_now2:
                if left[0] < x1 and abs(y1 - y2) > 10:
                    left[0] = x1
                    left[1] = y1
                    left[2] = x2
                    left[3] = y2
                    rear_left_out = 0
                    rear_right_out += 1
            else:
                if x1 < right[0] and abs(y1 - y2) > 10:
                    right[0] = x1
                    right[1] = y1
                    right[2] = x2
                    right[3] = y2
                    rear_right_out = 0
                    rear_left_out += 1
            # cv2.line(rear_warped_frame, (x1, y1), (x2, y2), (51, 104, 255), 2)  # 인식 확인

        for i in range(0, 4):
            left_lane2[i] = left[i]
            right_lane2[i] = right[i]

    else:
        rear_right_out += 1
        rear_left_out += 1

    draw_line2()

    x_now2 = int((x_1_l2 + x_1_r2) / 2)

    # 후면 주차 평행 인식
    if car_mode == 2:
        if reverse_lines is not None:
            for line in reverse_lines:
                x1, y1, x2, y2 = line[0]
                if abs(y1 - y2) < 5 and reverse_chance == 7:
                    dead_line_count += 1
                    if dead_line_count > max_dead_line_count:
                        dead_line_count = max_dead_line_count
                    # cv2.line(rear_warped_frame, (x1, y1), (x2, y2), (51, 104, 255), 2)  # 인식 확인

# warp된 영상에 인식된 정보 표기
def road_display():
    global front_warped_frame
    global rear_warped_frame
    global front_frame
    global rear_frame
    half = int(width / 2)
    if ((car_mode == 0 or car_mode == 3) and (abs(x_1_l - half) > 3 or abs(x_1_r - half) > 3 or abs(x_2_l - half) > 3 or abs(x_2_r - half) > 3)) or car_mode == 1 or car_mode == 4:
        cv2.line(front_warped_frame, (x_1_l, height - 120), (x_2_l, int(height / 2)), (255, 231, 40), 3)
        cv2.line(front_warped_frame, (x_1_r, height - 120), (x_2_r, int(height / 2)), (255, 231, 40), 3)
        cv2.line(rear_warped_frame, (x_1_l2, height - 120), (x_2_l2, int(height / 2)), (255, 231, 40), 3)
        cv2.line(rear_warped_frame, (x_1_r2, height - 120), (x_2_r2, int(height / 2)), (255, 231, 40), 3)

        cv2.line(front_warped_frame, (int((x_1_l + x_1_r) / 2), height - 120), (int((x_2_l + x_2_r) / 2), int(height / 2)), (255, 231, 40), 3)  # delete
        cv2.line(rear_warped_frame, (int((x_1_l2 + x_1_r2) / 2), height - 120), (int((x_2_l2 + x_2_r2) / 2), int(height / 2)), (255, 231, 40), 3)  # delete

        cv2.line(front_warped_frame, (int(width / 2), height - 120), (int(width / 2), int(height / 2)), (255, 0, 0), 2)
        cv2.line(rear_warped_frame, (int(width / 2), height - 120), (int(width / 2), int(height / 2)), (255, 0, 0), 2)

        front_vertices_road = np.array([[(x_1_l, height - 120), (x_2_l, height / 2), (x_2_r, height / 2), (x_1_r, height - 120)]], dtype=np.int32)
        front_road_frame = front_warped_frame.copy()
        front_road = cv2.fillPoly(front_road_frame, [front_vertices_road], (255, 191, 0))
        rear_vertices_road = np.array([[(x_1_l2, height - 120), (x_2_l2, height / 2), (x_2_r2, height / 2), (x_1_r2, height - 120)]], dtype=np.int32)
        rear_road_frame = rear_warped_frame.copy()
        rear_road = cv2.fillPoly(rear_road_frame, [rear_vertices_road], (255, 191, 0))

        front_warped_frame = cv2.addWeighted(front_warped_frame, 0.4, front_road, 0.6, 0)
        rear_warped_frame = cv2.addWeighted(rear_warped_frame, 0.4, rear_road, 0.6, 0)

    elif car_mode == 2:
        rear_tire_height = int(120 * math.sin(math.radians(abs(real_rear_steer))))
        rear_tire_width = int(-120 * math.cos(math.radians(abs(real_rear_steer))))

        cv2.line(rear_warped_frame, (270, 360), (270 - rear_tire_width, 360 - rear_tire_height), (0, 255, 255), 5)
        cv2.line(rear_warped_frame, (width - 270, 360), (width - 270 - rear_tire_width, 360 - rear_tire_height), (0, 255, 255), 5)

        # -----------------------------------------------------------------------------------------

    m = cv2.getPerspectiveTransform(dst, src)
    front_made_frame = cv2.warpPerspective(front_warped_frame, m, (width, height))
    cv2.fillPoly(front_frame, [cut], (0, 0, 0))
    front_frame = cv2.add(front_frame, front_made_frame)
    m2 = cv2.getPerspectiveTransform(dst, src)
    rear_made_frame = cv2.warpPerspective(rear_warped_frame, m2, (width, height))
    cv2.fillPoly(rear_frame, [cut], (0, 0, 0))
    rear_frame = cv2.add(rear_frame, rear_made_frame)

    """
     if car_mode == 2:
            cv2.polylines(rear_warped_frame, [reverse_vertices], True, (0, 0, 255), 2)
        else:
            cv2.polylines(front_warped_frame, [front_vertices], True, (153, 153, 153), 2)
            cv2.polylines(rear_warped_frame, [rear_vertices], True, (153, 153, 153), 2)
    """


# 거리에 따른 초음파 표기
def sonic_display(result):
    global sound_num
    result[365:365 + 230, 920:920 + 80] = all_car_pic
    sonic = result.copy()
    colors = [0, 0, 0, 0, 0, 0]

    for i in range(0, 6):
        if real_distances[i] < dis_4:
            colors[i] = (60, 20, 220)
        elif real_distances[i] < dis_3:
            colors[i] = (0, 165, 255)
        elif real_distances[i] < dis_2:
            colors[i] = (51, 255, 255)
        elif real_distances[i] < dis_1:
            colors[i] = (50, 205, 50)

    # front
    if real_distances[0] < dis_1:
        if real_distances[0] > dis_2:
            cv2.ellipse(sonic, (960, 350), (90, 90), 0, 220, 320, colors[0], 15)
        if real_distances[0] > dis_3:
            cv2.ellipse(sonic, (960, 350), (70, 70), 0, 220, 320, colors[0], 15)
        if real_distances[0] > dis_4:
            cv2.ellipse(sonic, (960, 350), (50, 50), 0, 220, 320, colors[0], 15)
        cv2.ellipse(sonic, (960, 350), (30, 30), 0, 220, 320, colors[0], 15)

    if real_distances[2] < dis_1:
        if real_distances[2] > dis_2:
            cv2.ellipse(sonic, (920, 380), (90, 90), 0, 130, 230, colors[2], 15)
        if real_distances[2] > dis_3:
            cv2.ellipse(sonic, (920, 380), (70, 70), 0, 130, 230, colors[2], 15)
        if real_distances[2] > dis_4:
            cv2.ellipse(sonic, (920, 380), (50, 50), 0, 130, 230, colors[2], 15)
        cv2.ellipse(sonic, (920, 380), (30, 30), 0, 130, 230, colors[2], 15)

    if real_distances[3] < dis_1:
        if real_distances[3] > dis_2:
            cv2.ellipse(sonic, (1000, 380), (90, 90), 0, -50, 50, colors[3], 15)
        if real_distances[3] > dis_3:
            cv2.ellipse(sonic, (1000, 380), (70, 70), 0, -50, 50, colors[3], 15)
        if real_distances[3] > dis_4:
            cv2.ellipse(sonic, (1000, 380), (50, 50), 0, -50, 50, colors[3], 15)
        cv2.ellipse(sonic, (1000, 380), (30, 30), 0, -50, 50, colors[3], 15)

    # rear
    if real_distances[1] < dis_1:
        if real_distances[1] > dis_2:
            cv2.ellipse(sonic, (960, 600), (90, 90), 0, 40, 140, colors[1], 15)
        if real_distances[1] > dis_3:
            cv2.ellipse(sonic, (960, 600), (70, 70), 0, 40, 140, colors[1], 15)
        if real_distances[1] > dis_4:
            cv2.ellipse(sonic, (960, 600), (50, 50), 0, 40, 140, colors[1], 15)

        cv2.ellipse(sonic, (960, 600), (30, 30), 0, 40, 140, colors[1], 15)
    if real_distances[4] < dis_1:
        if real_distances[4] > dis_2:
            cv2.ellipse(sonic, (920, 580), (90, 90), 0, 130, 230, colors[4], 15)
        if real_distances[4] > dis_3:
            cv2.ellipse(sonic, (920, 580), (70, 70), 0, 130, 230, colors[4], 15)
        if real_distances[4] > dis_4:
            cv2.ellipse(sonic, (920, 580), (50, 50), 0, 130, 230, colors[4], 15)
        cv2.ellipse(sonic, (920, 580), (30, 30), 0, 130, 230, colors[4], 15)

    if real_distances[5] < dis_1:
        if real_distances[5] > dis_2:
            cv2.ellipse(sonic, (1000, 580), (90, 90), 0, -50, 50, colors[5], 15)
        if real_distances[5] > dis_3:
            cv2.ellipse(sonic, (1000, 580), (70, 70), 0, -50, 50, colors[5], 15)
        if real_distances[5] > dis_4:
            cv2.ellipse(sonic, (1000, 580), (50, 50), 0, -50, 50, colors[5], 15)
        cv2.ellipse(sonic, (1000, 580), (30, 30), 0, -50, 50, colors[5], 15)

    for i in range(0, 6):
        if real_distances[i] < dis_4 and (sound_num == 0 or sound_num == 4):
            sound_num = dis_4
        elif real_distances[i] < dis_3 and (sound_num == 0 or sound_num == 4):
            sound_num = dis_3
        elif real_distances[i] < dis_2 and (sound_num == 0 or sound_num == 4):
            sound_num = dis_2
        elif real_distances[i] < dis_1 and (sound_num == 0 or sound_num == 4):
            sound_num = dis_1
    result = cv2.addWeighted(sonic, 0.7, result, 0.3, 0)
    return result


# 총 정리된 영상 출력
def video_play():
    global all_car_pic
    global rear_frame

    cv2.namedWindow("GARAGE", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("GARAGE", 1300, 750)
    # cv2.resizeWindow("GARAGE", 800, 500)

    cv2.moveWindow("GARAGE", 100, 30)
    turn = cv2.flip(rear_warped_frame, 0)

    front = cv2.hconcat([front_frame, front_warped_frame])
    rear = cv2.hconcat([rear_frame, turn])
    total = cv2.vconcat([front, rear])
    result = cv2.hconcat([total, state_video])

    result = sonic_display(result)

    cv2.imshow("GARAGE", result)


# -----------------------------------------------------------------------------------------


# 상태창(가장 왼쪽)에 나타나는 화면 
def top_view():
    global state_video
    global top_car_pic
    global all_car_pic
    m_h = int(s_height / 2 - 50)

    width_s = int(40 * math.cos(math.radians(abs(real_middle - 180))))
    height_s = int(40 * math.sin(math.radians(abs(real_middle - 180))))

    if real_middle > 180:
        width_m = int(width / 2) + int(340 * math.sin(math.radians(abs(real_middle - 180))))
        height_m = m_h + int(340 * math.cos(math.radians(abs(real_middle - 180))))

        f_l = (int(width / 2) - width_s, m_h + height_s)
        f_r = (int(width / 2) + width_s, m_h - height_s)
        r_l = (width_m - width_s, height_m + height_s)
        r_r = (width_m + width_s, height_m - height_s)
    else:
        width_m = int(width / 2) - int(340 * math.sin(math.radians(abs(real_middle - 180))))
        height_m = m_h + int(340 * math.cos(math.radians(abs(real_middle - 180))))

        f_l = (int(width / 2) - width_s, m_h - height_s)
        f_r = (int(width / 2) + width_s, m_h + height_s)
        r_l = (width_m - width_s, height_m - height_s)
        r_r = (width_m + width_s, height_m + height_s)

    vertices_state = np.array([[f_l, f_r, r_r, r_l]], dtype=np.int32)

    # state_video = cv2.rectangle(state_video, (int(width / 2 - 50), 300), (int(width / 2 + 50), int(s_height / 2+10)), (240, 240, 240), -1)
    state_video = cv2.fillPoly(state_video, [vertices_state], (220, 222, 222))
    state_video = cv2.polylines(state_video, [vertices_state], True, (149, 151, 151), 5)

    top_car_pic = cv2.resize(top_car_pic, (102, 200))

    state_video[300:300 + 200, int(width / 2 - 50):int(width / 2 - 50) + 102] = top_car_pic


# 상태창에 나타나는 타이어
def tire():
    global real_middle
    global real_rear_steer

    front_tire_height = int(20 * math.sin(math.radians(abs(real_front_steer))))
    front_tire_width = int(-20 * math.cos(math.radians(abs(real_front_steer))))
    front_tire_road_height = int(80 * math.sin(math.radians(abs(real_front_steer))))
    front_tire_road_width = int(-150 * math.cos(math.radians(abs(real_front_steer))))

    rear_tire_height = int(20 * math.sin(math.radians(abs(real_rear_steer - (real_middle - 180)))))
    rear_tire_width = int(-20 * math.cos(math.radians(abs(real_rear_steer - (real_middle - 180)))))
    rear_tire_road_height = int(80 * math.sin(math.radians(abs(real_rear_steer - (real_middle - 180)))))
    rear_tire_road_width = int(-150 * math.cos(math.radians(abs(real_rear_steer - (real_middle - 180)))))

    trie_size = 20
    road_size = 10

    if not receive_check or (front_right_out > min_out and front_left_out > min_out):
        tire_color = (0, 165, 255)
    else:
        tire_color = (150, 150, 150)

    if car_mode == 1:
        cv2.line(state_video, (270, 350), (270 + front_tire_road_width, 350 - front_tire_road_height), (0, 255, 255), road_size)
        cv2.line(state_video, (width - 270, 350), (width - 270 + front_tire_road_width, 350 - front_tire_road_height), (0, 255, 255), road_size)
    elif car_mode == 2:
        cv2.line(state_video, (270, 350), (270 - front_tire_road_width, 350 + front_tire_road_height), (0, 255, 255), road_size)
        cv2.line(state_video, (width - 270, 350), (width - 270 - front_tire_road_width, 350 + front_tire_road_height), (0, 255, 255), road_size)

    cv2.line(state_video, (270, 350), (270 + front_tire_width, 350 - front_tire_height), tire_color, trie_size)
    cv2.line(state_video, (270, 350), (270 - front_tire_width, 350 + front_tire_height), tire_color, trie_size)
    cv2.line(state_video, (width - 270, 350), (width - 270 + front_tire_width, 350 - front_tire_height), tire_color, trie_size)
    cv2.line(state_video, (width - 270, 350), (width - 270 - front_tire_width, 350 + front_tire_height), tire_color, trie_size)

    cv2.line(state_video, (270, 410), (270, 450), tire_color, trie_size)
    cv2.line(state_video, (width - 270, 410), (width - 270, 450), tire_color, trie_size)

    m_h = int(s_height / 2 - 50)

    width_s = int(50 * math.cos(math.radians(abs(real_middle - 180))))
    height_s = int(50 * math.sin(math.radians(abs(real_middle - 180))))
    if real_middle > 180:
        width_m = int(width / 2) + int(300 * math.sin(math.radians(abs(real_middle - 180))))
        height_m = m_h + int(300 * math.cos(math.radians(abs(real_middle - 180))))
        if car_mode == 1:
            cv2.line(state_video, (width_m + width_s, height_m - height_s), (width_m + width_s + rear_tire_road_width, height_m - height_s - rear_tire_road_height), (0, 255, 255), road_size)
            cv2.line(state_video, (width_m - width_s, height_m + height_s), (width_m - width_s + rear_tire_road_width, height_m - height_s - rear_tire_road_height), (0, 255, 255), road_size)
        elif car_mode == 2:
            cv2.line(state_video, (width_m + width_s, height_m - height_s), (width_m + width_s - rear_tire_road_width, height_m - height_s + rear_tire_road_height), (0, 255, 255), road_size)
            cv2.line(state_video, (width_m - width_s, height_m + height_s), (width_m - width_s - rear_tire_road_width, height_m - height_s + rear_tire_road_height), (0, 255, 255), road_size)

        cv2.line(state_video, (width_m - width_s, height_m + height_s), (width_m - width_s + rear_tire_width, height_m + height_s - rear_tire_height), tire_color, trie_size)
        cv2.line(state_video, (width_m - width_s, height_m + height_s), (width_m - width_s - rear_tire_width, height_m + height_s + rear_tire_height), tire_color, trie_size)
        cv2.line(state_video, (width_m + width_s, height_m - height_s), (width_m + width_s + rear_tire_width, height_m - height_s - rear_tire_height), tire_color, trie_size)
        cv2.line(state_video, (width_m + width_s, height_m - height_s), (width_m + width_s - rear_tire_width, height_m - height_s + rear_tire_height), tire_color, trie_size)

    else:
        width_m = int(width / 2) - int(300 * math.sin(math.radians(abs(real_middle - 180))))
        height_m = m_h + int(300 * math.cos(math.radians(abs(real_middle - 180))))
        if car_mode == 1:
            cv2.line(state_video, (width_m + width_s, height_m + height_s), (width_m + width_s + rear_tire_road_width, height_m - height_s - rear_tire_road_height), (0, 255, 255), road_size)
            cv2.line(state_video, (width_m - width_s, height_m - height_s), (width_m - width_s + rear_tire_road_width, height_m - height_s - rear_tire_road_height), (0, 255, 255), road_size)
        elif car_mode == 2:
            cv2.line(state_video, (width_m + width_s, height_m + height_s), (width_m + width_s - rear_tire_road_width, height_m - height_s + rear_tire_road_height), (0, 255, 255), road_size)
            cv2.line(state_video, (width_m - width_s, height_m - height_s), (width_m - width_s - rear_tire_road_width, height_m - height_s + rear_tire_road_height), (0, 255, 255), road_size)

        cv2.line(state_video, (width_m - width_s, height_m - height_s), (width_m - width_s + rear_tire_width, height_m - height_s - rear_tire_height), tire_color, trie_size)
        cv2.line(state_video, (width_m - width_s, height_m - height_s), (width_m - width_s - rear_tire_width, height_m - height_s + rear_tire_height), tire_color, trie_size)
        cv2.line(state_video, (width_m + width_s, height_m + height_s), (width_m + width_s + rear_tire_width, height_m + height_s - rear_tire_height), tire_color, trie_size)
        cv2.line(state_video, (width_m + width_s, height_m + height_s), (width_m + width_s - rear_tire_width, height_m + height_s + rear_tire_height), tire_color, trie_size)

# 상태창에 나타나는 차선
def lane():
    if car_mode == 0 or car_mode == 1 or car_mode == 4:
        lane_size = 15
        if car_mode == 1:
            lane_color = (227, 186, 67)
        else:
            lane_color = (150, 150, 150)

        m_h = int(s_height / 2 - 50)

        width_s = int(100 * math.cos(math.radians(abs(real_middle - 180))))
        height_s = int(100 * math.sin(math.radians(abs(real_middle - 180))))

        if real_middle >= 180:
            l_1 = [x_2_l - 100, 200]
            l_2 = [x_1_l - 100, 300]
            l_3 = [x_1_l - 100, int(s_height / 2) - 100]

            width_m = int(width / 2 - 320) + int(80 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(100 * math.cos(math.radians(abs(real_middle - 180))))
            l_4 = [x_1_l2 + width_m - width_s, height_m + height_s]

            width_m = int(width / 2 - 320) + int(350 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(350 * math.cos(math.radians(abs(real_middle - 180))))
            l_5 = [x_1_l2 + width_m - width_s, height_m + height_s]

            width_m = int(width / 2 - 320) + int(450 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(450 * math.cos(math.radians(abs(real_middle - 180))))
            l_6 = [x_2_l2 + width_m - width_s, height_m + height_s]

            r_1 = [x_2_r + 100, 200]
            r_2 = [x_1_r + 100, 300]
            r_3 = [x_1_r + 100, int(s_height / 2 - 100)]

            width_m = int(width / 2 - 320) + int(80 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(100 * math.cos(math.radians(abs(real_middle - 180))))
            r_4 = [x_1_r2 + width_m + width_s, height_m - height_s]

            width_m = int(width / 2 - 320) + int(350 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(350 * math.cos(math.radians(abs(real_middle - 180))))
            r_5 = [x_1_r2 + width_m + width_s, height_m - height_s]

            width_m = int(width / 2 - 320) + int(450 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(450 * math.cos(math.radians(abs(real_middle - 180))))
            r_6 = [x_2_r2 + width_m + width_s, height_m - height_s]

        else:
            l_1 = [x_2_l - 100, 200]
            l_2 = [x_1_l - 100, 300]
            l_3 = [x_1_l - 100, int(s_height / 2 - 100)]

            width_m = int(width / 2 - 320) - int(80 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(100 * math.cos(math.radians(abs(real_middle - 180))))
            l_4 = [x_1_l2 + width_m - width_s, height_m - height_s]

            width_m = int(width / 2 - 320) - int(350 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(350 * math.cos(math.radians(abs(real_middle - 180))))
            l_5 = [x_1_l2 + width_m - width_s, height_m - height_s]

            width_m = int(width / 2 - 320) - int(450 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(450 * math.cos(math.radians(abs(real_middle - 180))))
            l_6 = [x_2_l2 + width_m - width_s, height_m - height_s]

            r_1 = [x_2_r + 100, 200]
            r_2 = [x_1_r + 100, 300]
            r_3 = [x_1_r + 100, int(s_height / 2 - 100)]

            width_m = int(width / 2 - 320) - int(80 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(100 * math.cos(math.radians(abs(real_middle - 180))))
            r_4 = [x_1_r2 + width_m + width_s, height_m + height_s]

            width_m = int(width / 2 - 320) - int(350 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(350 * math.cos(math.radians(abs(real_middle - 180))))
            r_5 = [x_1_r2 + width_m + width_s, height_m + height_s]

            width_m = int(width / 2 - 320) - int(450 * math.sin(math.radians(abs(real_middle - 180))))
            height_m = m_h + int(450 * math.cos(math.radians(abs(real_middle - 180))))
            r_6 = [x_2_r2 + width_m + width_s, height_m + height_s]

        l_vertices_lane = np.array([l_1, l_2, l_3, l_4, l_5, l_6], dtype=np.int32)
        r_vertices_lane = np.array([r_1, r_2, r_3, r_4, r_5, r_6], dtype=np.int32)

        cv2.polylines(state_video, [l_vertices_lane], False, lane_color, lane_size)
        cv2.polylines(state_video, [r_vertices_lane], False, lane_color, lane_size)


# 상태창 표기
def state_display():
    global speed
    global speed_location
    global sound_num
    global state_video

    state_video = np.zeros((height * 2, width, 3), np.uint8)
    top_view()
    tire()
    lane()

    if speed >= 100:
        speed_location = (int(width / 2) - 55, 100)
    else:
        speed_location = (int(width / 2) - 25, 100)
    cv2.putText(state_video, str(int(speed / 10)), speed_location, font, 3, (230, 230, 230), 6)

    if car_mode == 0:
        cv2.putText(state_video, "P", mode_location, font, 2, (50, 205, 50), 5)
        if not receive_check or (front_left_out > min_out and front_right_out > min_out):
            cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (60, 20, 220), 2)
        else:
            cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (250, 250, 250), 2)
    elif car_mode == 1:
        cv2.putText(state_video, "D", mode_location, font, 2, (255, 255, 255), 5)
        cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (50, 205, 50), 2)
    elif car_mode == 2:
        if direction == 0:
            cv2.putText(state_video, "D", mode_location, font, 2, (255, 255, 255), 5)
            cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (50, 205, 50), 2)
        else:
            cv2.putText(state_video, "R", mode_location, font, 2, (60, 20, 220), 5)
            cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (50, 205, 50), 2)
    elif car_mode == 3:
        cv2.putText(state_video, "S", mode_location, font, 2, (255, 231, 40), 5)
        cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (50, 205, 50), 2)
    elif car_mode == 11:
        cv2.putText(state_video, "D", mode_location, font, 2, (255, 255, 255), 5)
        cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (250, 250, 250), 2)
    elif car_mode == 22:
        cv2.putText(state_video, "R", mode_location, font, 2, (60, 20, 220), 5)
        cv2.putText(state_video, "AUTO", autopilot_location, font, fontScale, (250, 250, 250), 2)

    #cv2.putText(state_video, "R:" + str(reverse_chance), (int(width / 2) + 170, 200), font, fontScale, (50, 205, 50), 2)

    #cv2.putText(state_video, "A:" + str(real_front_steer), (int(width / 2) + 170, 250), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "P:" + str(round(front_p, 2)), (int(width / 2) + 170, 300), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "I:" + str(round(front_i, 2)), (int(width / 2) + 170, 350), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "D:" + str(round(front_d, 2)), (int(width / 2) + 170, 400), font, fontScale, (50, 205, 50), 2)

    #cv2.putText(state_video, "M:" + str(real_middle), (int(width / 2) + 170, 500), font, fontScale, (50, 205, 50), 2)

    #cv2.putText(state_video, "B:" + str(real_rear_steer), (int(width / 2) + 170, 600), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "P:" + str(round(rear_p, 2)), (int(width / 2) + 170, 650), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "I:" + str(round(rear_i, 2)), (int(width / 2) + 170, 700), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "D:" + str(round(rear_d, 2)), (int(width / 2) + 170, 750), font, fontScale, (50, 205, 50), 2)

    #cv2.putText(state_video, "S:" + str(parking_chance), (0, 200), font, fontScale, (50, 205, 50), 2)

    #cv2.putText(state_video, "P:" + str(front_Kp), (0, 250), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "I:" + str(front_Ki), (0, 300), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "D:" + str(front_Kd), (0, 350), font, fontScale, (50, 205, 50), 2)

    #cv2.putText(state_video, "P:" + str(rear_Kp), (0, 600), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "I:" + str(rear_Ki), (0, 650), font, fontScale, (50, 205, 50), 2)
    #cv2.putText(state_video, "D:" + str(rear_Kd), (0, 700), font, fontScale, (50, 205, 50), 2)


# -----------------------------------------------------------------------------------------

# 사운드 스레드
def sound():
    global sound_num
    playsound("welcome_sound.mp3")
    while True:
        if sound_num == 1:
            playsound("horn.mov")
            playsound("watch_out.mp3")
            if sound_num == 1:
                sound_num = 0
        elif sound_num == 2:
            playsound("horn.mov")
            if sound_num == 2:
                sound_num = 0
        elif sound_num == 3:
            playsound("short_horn.mov")
            if sound_num == 3:
                sound_num = 0
        elif sound_num == 4:
            playsound("reverse_sound.mov")
            if sound_num == 4:
                sound_num = 0
        elif sound_num == 5:
            playsound("Auto_Start.mp3")
            if sound_num == 5:
                sound_num = 0
        elif sound_num == 6:
            playsound("Auto_End.mp3")
            if sound_num == 6:
                sound_num = 0
        elif sound_num == 7:
            playsound("Searching_Start.mp3")
            if sound_num == 7:
                sound_num = 0
        elif sound_num == 8:
            playsound("Searching_End.mp3")
            if sound_num == 8:
                sound_num = 0
        elif sound_num == 9:
            playsound("Parking_Start.mp3")
            if sound_num == 9:
                sound_num = 0
        elif sound_num == 10:
            playsound("Parking_End.mp3")
            if sound_num == 10:
                sound_num = 0
        elif sound_num == 11:
            playsound("Parking_Finish.mp3")
            if sound_num == 11:
                sound_num = 0
        elif sound_num == 12:
            playsound("drive.mp3")
            if sound_num == 12:
                sound_num = 0
        elif sound_num == 13:
            playsound("reverse.mp3")
            if sound_num == 13:
                sound_num = 0
        elif sound_num == 14:
            playsound("stop.mp3")
            if sound_num == 14:
                sound_num = 0
        elif sound_num == 15:
            playsound("emergency_stop.mp3")
            if sound_num == 15:
                sound_num = 0
        elif sound_num == 16:
            playsound("lane_check.mp3")
            if sound_num == 16:
                sound_num = 0
        elif sound_num == 17:
            playsound("reset.mp3")
            if sound_num == 17:
                sound_num = 0
        elif sound_num >= dis_4:
            playsound("sonic.wav")
            if sound_num == dis_1:
                time.sleep(0.1)
            elif sound_num == dis_2:
                time.sleep(0.05)
            elif sound_num == dis_3:
                time.sleep(0.02)
            if sound_num >= dis_4:
                sound_num = 0

        else:
            time.sleep(0.00001)


# 기능 종료
def stop_control():
    global car_mode
    global sound_num

    if car_mode == 1:
        print("Auto Pilot End")
        sound_num = 6
        car_mode = 0
        steering_reset()
    elif car_mode == 2:
        print("Parking End")
        sound_num = 10
        car_mode = 0
        steering_reset()
    elif car_mode == 3:
        print("Area Search End")
        car_mode = 0
        sound_num = 8
        steering_reset()
    elif car_mode == 4:
        car_mode = 0


# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------

# 아두이노와 통신 동기화
# Arduino    Python
#         <-   A
#   Y/N   ->   Connect/Not Connect
while True:
    op = "A"
    ser.write(op.encode('utf-8'))
    if ser.readable():
        try:
            data = ser.readline()
            data = data.decode()
            if Connect_Wait == 50:
                print("Not Connect")
                Connect_Wait = 0
            if data[0] == 'N':
                Connect_Wait += 1
            elif data[0] == 'Y':
                print("Connect")
                break
        except ValueError:
            print("receive_fail")

# -----------------------------------------------------------------------------------------

# 시리얼 통신(수신) 스레드
serial_thread = threading.Thread(target=serial_receive, args=())
serial_thread.daemon = True
serial_thread.start()
# 사운드 스레드
sound_thread = threading.Thread(target=sound, args=())
sound_thread.daemon = True
sound_thread.start()

# -----------------------------------------------------------------------------------------

while True:
    ret, front_frame = front_video.read()
    if ret == 0:
        print("No Front Camera")
        break

    ret, rear_frame = rear_video.read()
    if ret == 0:
        print("No Rear Camera")
        break

    # -----------------------------------------------------------------------------------------

    # 이미지 크기 조절
    front_frame = cv2.resize(front_frame, (width, height))
    rear_frame = cv2.resize(rear_frame, (width, height))

    # Perspective Transform (Bird`s Eye View)
    M = cv2.getPerspectiveTransform(src, dst)
    front_warped_frame = cv2.warpPerspective(front_frame, M, (width, height))
    M2 = cv2.getPerspectiveTransform(src, dst)
    rear_warped_frame = cv2.warpPerspective(rear_frame, M2, (width, height))

    """
# Gray
    front_gray_frame = cv2.cvtColor(front_warped_frame, cv2.COLOR_BGR2GRAY)
    front_darkened_frame = adjust_gamma(front_gray_frame, 0.5)
    rear_gray_frame = cv2.cvtColor(rear_warped_frame, cv2.COLOR_BGR2GRAY)
    rear_darkened_frame = adjust_gamma(rear_gray_frame, 0.5)

    # Gaussian
    front_gaussian_frame = cv2.GaussianBlur(front_gray_frame, (5, 5), 0)
    rear_gaussian_frame = cv2.GaussianBlur(rear_gray_frame, (5, 5), 0)
    """
    front_edges_frame = cv2.Canny(front_warped_frame, 100, 200)  # Canny
    rear_edges_frame = cv2.Canny(rear_warped_frame, 100, 200)  # Canny

    front_roi_frame = roi(front_edges_frame, [front_vertices])
    rear_roi_frame = roi(rear_edges_frame, [rear_vertices])
    reverse_roi_frame = roi(rear_edges_frame, [reverse_vertices])

    front_roi_frame = np.uint8(front_roi_frame)
    rear_roi_frame = np.uint8(rear_roi_frame)
    reverse_roi_frame = np.uint8(reverse_roi_frame)

    front_lines = cv2.HoughLinesP(front_roi_frame, 1, np.pi / 180, 20, minLineLength=5, maxLineGap=10)
    rear_lines = cv2.HoughLinesP(rear_roi_frame, 1, np.pi / 180, 20, minLineLength=5, maxLineGap=10)
    if car_mode == 2:
        reverse_lines = cv2.HoughLinesP(reverse_roi_frame, 1, np.pi / 180, 25, minLineLength=10, maxLineGap=100)

    # -----------------------------------------------------------------------------------------

    line_process()
    road_display()

    car_control()
    send_data()

    state_display()
    video_play()

    key = cv2.waitKey(1)

    if key == 32:  # 스페이스바
        op_end = "B"
        ser.write(op_end.encode('utf-8'))
        break
    elif key == 91 and (front_left_out < min_out or front_right_out < min_out):  # [:자율주행 시작
        if car_mode == 0:
            print("Auto Pilot Start")
            sound_num = 5
            car_mode = 1
            x_now = int(width / 2)
            x_now2 = int(width / 2)
            x_1_l = int(width / 2)
            x_2_l = int(width / 2)
            x_1_r = int(width / 2)
            x_2_r = int(width / 2)
            x_1_l2 = int(width / 2)
            x_2_l2 = int(width / 2)
            x_1_r2 = int(width / 2)
            x_2_r2 = int(width / 2)
        else:
            stop_control()
        pid_reset()
        steering_reset()
    elif key == 93:  # ]:주차 공간 탐색
        parking_chance = 0
        if car_mode == 0 or car_mode == 1:
            print("Area Search Start")
            sound_num = 7
            car_mode = 3
        else:
            stop_control()
        pid_reset()
    elif key == 92:  # \:후진 주차
        reverse_chance = 0
        dead_line_count = 0
        if car_mode == 0:
            print("Parking Start")
            sound_num = 9
            real_middle = 180
            op_r = "R"
            ser.write(op_r.encode('utf-8'))
            car_mode = 2
        else:
            stop_control()
        steering_reset()
    elif key == 102:  # f:스티어링 초기화
        print("Reset Steering")
        sound_num = 17
        steering_reset()
        x_now = int(width / 2)
        x_now2 = int(width / 2)
        x_1_l = int(width / 2)
        x_2_l = int(width / 2)
        x_1_r = int(width / 2)
        x_2_r = int(width / 2)
        x_1_l2 = int(width / 2)
        x_2_l2 = int(width / 2)
        x_1_r2 = int(width / 2)
        x_2_r2 = int(width / 2)
        real_middle = 180
        middle_sum = 0
        with middle_queue.mutex:
            middle_queue.queue.clear()
        op_r = "R"
        ser.write(op_r.encode('utf-8'))
    elif key == 115:  # s:전진
        if car_mode == 0:
            print("Drive")
            direction = 0
            manual_speed = 120
            sound_num = 12
            car_mode = 11
        elif car_mode == 11:
            print("Speed Up")
            manual_speed += 20
            if manual_speed >= 240:
                manual_speed = 240
        elif car_mode == 22:
            print("Speed Down")
            manual_speed -= 20
            if manual_speed < 120:
                manual_speed = 0
                sound_num = 14
                car_mode = 0
    elif key == 120:  # x:후진
        if car_mode == 0 or 11 or 22:
            if car_mode == 0:
                print("Reverse")
                direction = 1
                manual_speed = 120
                sound_num = 13
                car_mode = 22
            elif car_mode == 11:
                print("Speed Down")
                manual_speed -= 20
                if manual_speed < 120:
                    manual_speed = 0
                    sound_num = 14
                    car_mode = 0
            elif car_mode == 22:
                print("Speed Up")
                manual_speed += 20
                if manual_speed >= 240:
                    manual_speed = 240
    elif key == 118:  # v:정지
        print("STOP")
        sound_num = 14
        car_mode = 0
        steering_reset()
    elif key == 97 and (car_mode == 0 or 11 or 22) and manual_front_steer_count > -30:  # a:전륜 좌회전
        print("Front_Left")
        manual_front_steer_count -= 1
        manual_steer = 1
        if manual_front_steer_count == -30:
            sound_num = 3
    elif key == 100 and (car_mode == 0 or car_mode == 11 or car_mode == 22) and manual_front_steer_count < 30:  # d:전륜 우회전
        print("Front_Right")
        manual_front_steer_count += 1
        manual_steer = 2
        if manual_front_steer_count == 30:
            sound_num = 3
    elif key == 103 and (car_mode == 0 or car_mode == 11 or car_mode == 22) and manual_front_steer_count > -30 and manual_rear_steer_count < 30:  # g:좌회전
        print("Left")
        manual_front_steer_count -= 1
        manual_rear_steer_count += 1
        manual_steer = 5
        if manual_front_steer_count == -30 or manual_rear_steer_count == 30:
            sound_num = 3
    elif key == 106 and (car_mode == 0 or car_mode == 11 or car_mode == 22) and manual_front_steer_count < 30 and manual_rear_steer_count > -30:  # j:우회전
        print("Right")
        manual_front_steer_count += 1
        manual_rear_steer_count -= 1
        manual_steer = 6
        if manual_front_steer_count == 30 or manual_rear_steer_count == -30:
            sound_num = 3
    elif key == 122 and (car_mode == 0 or car_mode == 11 or car_mode == 22) and manual_rear_steer_count > -30:  # z:후륜 좌회전
        print("Rear_Left")
        manual_rear_steer_count -= 1
        manual_steer = 3
        if manual_rear_steer_count == -30:
            sound_num = 3
    elif key == 99 and (car_mode == 0 or car_mode == 11 or car_mode == 22) and manual_rear_steer_count < 30:  # c:후륜 우회전
        print("Rear_Right")
        manual_rear_steer_count += 1
        manual_steer = 4
        if manual_rear_steer_count == 30:
            sound_num = 3
    elif key == 104:  # h:혼
        sound_num = 1

    elif key == 49:
        pid_mode = 1
    elif key == 50:
        pid_mode = 2
    elif key == 51:
        pid_mode = 3
    elif key == 52:
        pid_mode = 4
    elif key == 53:
        pid_mode = 5
    elif key == 54:
        pid_mode = 6
    elif key == 61:
        pid_ud = 1
        pid_change()
    elif key == 45:
        pid_ud = 2
        pid_change()

    elif key == 116:  # t: 차선 유무
        if car_mode == 0:
            car_mode = 4
            sound_num = 16
        else:
            stop_control()

op_end = "B"
ser.write(op_end.encode('utf-8'))

front_video.release()
rear_video.release()
cv2.destroyAllWindows()