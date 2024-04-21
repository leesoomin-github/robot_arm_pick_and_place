import cv2
import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np

# 전역 변수 (현재 좌표)
current_actual = None


def connect_robot(ip):
    try:
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("연결 설정 중...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print("연결 성공!!")
        return dashboard, move, feed
    
    except Exception as e:
        print("연결 실패")
        raise e


def robot_clear(dashboard : DobotApiDashboard):
    dashboard.ClearError()


# 로봇 속도 조절 함수
def robot_speed(dashboard: DobotApiDashboard, speed_value):
    dashboard.SpeedFactor(speed_value)


# 그리퍼 구동 함수
def gripper_DO(dashboard: DobotApiDashboard, index, status):
    dashboard.ToolDO(index, status)


# 현재 로봇 위치 받아오기
def get_Pose(dashboard: DobotApiDashboard):
    dashboard.GetPose()


# 현재 위치 -> 목표 위치(point_list)
def run_point(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])


def get_feed(feed: DobotApi):
    global current_actual
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':
            current_actual = a["tool_vector_actual"][0]
        # Refresh Properties
        sleep(0.001)


# 로봇이 목표 위치로 도달할 때까지 기다리는 함수
def wait_arrive(point_list):
    global current_actual
    while True:
        is_arrive = True
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                return
        sleep(0.001)


# 입력 파라미터
ip = "192.168.1.6"
gripper_port = 1
speed_value = 100
# Robot의 IP 주소
# 그리퍼 포트 번호
# 로봇 속도 (1~100 사이의 값 입력)
# 로봇이 이동하고자 하는 좌표 (x, y, z, yaw) unit : mm, degree
point_home = [245, 5, 50, 115]       


# 로봇 연결
dashboard, move, feed = connect_robot(ip)
dashboard.EnableRobot()
print("이제 로봇을 사용할 수 있습니다!")

# 쓰레드 설정
feed_thread = threading.Thread(target=get_feed, args=(feed,))
feed_thread.setDaemon(True)
feed_thread.start()


# 구동 알고리즘 전역변수
y1_floor1 = [207.355705, 44.261897, -59.197216, 23.925671] # x, y, z, yaw
y1_floor2 = [206.878181, 43.086519, -42.711208, 23.803080]
y1_floor3 = [208.560156, 44.648763, -26.058420, 27.055161]

y2_floor1 = [296.641020, 110.782638, -59.679607, 38.122536]
y2_floor2 = [297.028896, 110.318359, -43.198338, 38.038574]
y2_floor3 = [295.664138, 109.262906, -26.869501, 37.953339]

g1_floor1 = [241.009502, -13.435659, -58.934727, 14.587836]
g1_floor2 = [242.370139, -14.204580, -42.780235, 14.509674]
g1_floor3 = [241.876009, -14.034321, -26.101986, 14.559511]

g2_floor1 = [341.082223, 53.488574, -59.733067, 29.091684]
g2_floor2 = [340.206709, 54.061793, -43.437149, 28.955750]
g2_floor3 = [338.544997, 55.546790, -27.073120, 28.980568]

g3_floor1 = [338.599341, -71.307019, -59.965996, 16.597919]
g3_floor2 = [339.876121, -70.615850, -42.811165, 16.714527]
g3_floor3 =  [339.318405, -70.358449, -27.285664, 17.286753]

r1_floor1 = [209.235748, -75.135154, -59.376179, 9.182369]
r1_floor2 = [208.968673, -73.360605, -42.522560, 9.574614]
r1_floor3 = [208.034250, -74.413086, -26.021393, 9.236820]

r2_floor1 = [295.269979, -122.978879, -59.536869, 6.669760]
r2_floor2 = [295.524728, -124.134638, -42.952744, 6.227392]
r2_floor3 = [294.889924, -124.186618, -26.720461, 6.229721]

stack_info_list = []
goal_point = (0,0)

webcam= cv2.VideoCapture(0)

if not webcam.isOpened():
    print("Could not open webcam")
    exit()

while webcam.isOpened():
    status, src = webcam.read()

    if status:
        dst = src.copy()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        ## 원 인식
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1 = 250, param2 = 10, minRadius = 80, maxRadius = 13)

        if len(circles[0]) > 1:
            print("circle is more than one!!!!!")
            for i in circles[0]:
                cv2.circle(dst, (i[0], i[1]), i[2], (255, 0, 0), 5)
        elif len(circles[0]) == 0:
            print("no circle!!!!!!")
        else:
            print("only one circle exist!!!!")
            goal_point = (circles[0][0], circles[0][1])

        for i in range(3):
            # 그리퍼 이동
            run_point(move, goal_point)
            sleep(0.1)

            # 그리퍼 구동
            gripper_DO(dashboard, gripper_port, 1)
            sleep(0.1)

            # 모양 색구별

            # 위치 이동

            # 그리퍼 해제
            gripper_DO(dashboard, gripper_port, 0)
            sleep(0.1)

            # stack_info_list에 append

            # if i == 2:
                # break
        
        for i in range (2):
            run_point(move, [stack_info_list[2-i]["x"], stack_info_list[2-i]["y"],
                             stack_info_list[2-i]["z"], stack_info_list[2-i]["yaw"]])
            sleep(0.1)

            gripper_DO(dashboard, gripper_port, 1)
            sleep(0.1)

            run_point(move, [stack_info_list[2]["x"], stack_info_list[2]["y"],
                             stack_info_list[2]["z"], stack_info_list[2]["yaw"]])
            sleep(0.1)

            gripper_DO(dashboard, gripper_port, 1)
            sleep(0.1)         

        cv2.imshow("dst", dst)
        if cv2.waitKey(1) == ord('q'):
            break

webcam.release()
cv2.destroyAllWindows()

    
