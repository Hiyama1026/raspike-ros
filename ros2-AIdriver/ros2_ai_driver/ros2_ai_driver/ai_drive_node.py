'''
自動運転ラジコンカー
AIドライバーあいちゃん
http://ma2.la.coocan.jp/AI_Driver/

走行プログラム
aidriver.py
author mitsuhiro matsuura
version 1.01
date 2023.01.22
'''

import rclpy
from raspike_uros_msg.msg  import MotorSpeedMessage
from raspike_uros_msg.msg import SpikeDevStatusMessage
from raspike_uros_msg.msg import ButtonStatusMessage
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from rclpy.node import Node

import sys

# passの設定 (pip showで出てきた、LocationのPASSを以下に設定)
#sys.path.append('/home/hlab/venv/lib/python3.10/site-packages')
#import cfg
import ros2_ai_driver.cfg as cfg
import concurrent.futures
import csv
import cv2
import multiprocessing
import numpy as np
import os
import struct
import tensorflow as tf
import time
from rclpy.qos import QoSProfile

q_cam = multiprocessing.Queue()					
q_img = multiprocessing.Queue(0)				
v_auto_on = multiprocessing.Value('b', False)		#AI操作のON/OFFを区別するための変数
v_esc = multiprocessing.Value('f', 0)				#縦の操作量
v_esc_on = multiprocessing.Value('b', False)		#モーター動作のON/OFFを格納する変数
v_fps = multiprocessing.Value('f', 30)				#fpsの値を格納する変数
v_log_on = multiprocessing.Value('b', False)		#log記録のON/OFFを格納する変数
v_log_print_on = multiprocessing.Value('b', False)	
count_reset = multiprocessing.Value('b', False)
v_angle_log_on = multiprocessing.Value('b', False)
v_model = multiprocessing.Value('b', False)			#モデルの読み込みができたかどうかを格納する変数
v_num = multiprocessing.Value('I', 0)				#記録したデータの数を格納する変数
v_ok = multiprocessing.Value('b', True)				#プログラムが正常に起動しているかの状態を格納する変数
v_servo = multiprocessing.Value('f', 0)				#横の操作量を格納する変数
v_servo_trim = multiprocessing.Value('f', cfg.SERVO_TRIM)
v_speed_trim = multiprocessing.Value('f', cfg.SPEED_TRIM)
v_speed_trim_ai = multiprocessing.Value('f', cfg.SPEED_TRIM_AI)
v_tick1 = multiprocessing.Value('I', 0)				
v_tick2 = multiprocessing.Value('I', 0)				
v_trigger = multiprocessing.Value('f', 0)			#スティックによる縦の操作量
v_wheel = multiprocessing.Value('f', 0)				#スティックによる横の操作量


# AIdriveノード
class AIdriveNode(Node):
		# 初期化
	def __init__(self):
		super().__init__("ai_drive_node")	   #コンストラクタ

		qos_profile = QoSProfile(depth=10, reliability=2)

		self.drive_is_first = True
		self.fpv = Fpv()												# fpvクラスの読み込み
		self.log = Log(cfg.LOG_DIR)									# ログ出力のセットアップ
		self.model_a = Model(cfg.MODEL_FILE_A)							# モデルがあるかの確認
		#self.model_b = Model(cfg.MODEL_FILE_B)
		#self.model_c = Model(cfg.MODEL_FILE_C)		
		self.speed = Speed(cfg.SPEED_TRIM, cfg.SPEED_TRIM_AI)		# スピードトリムの読み込み
		self.t1 = 0.0												
		self.t2 = 0.0												# 1周期あたりの時間(ms)
		self.t3 = 0.0												# 推論にかかる時間(ms)
		self.cnt = 0
		self.is_end = False
		self.end_count = 0
		self.r_init = 0
		self.r_current = 0
		self.l_init = 0
		self.l_current = 0
		self.model_mode = 0

		self.rev_x_angle = 0
		self.pre_rev_x_angle = 0
		self.pre_rev_z_angle = 0
		self.rev_z_angle = 0
		self.pre_button_val = 0
		self.rev_button_val = 0

		# パブリッシャーの生成 (引数：メッセージ型．トピック名，QoS設定)
		self.imu_init_publisher = self.create_publisher(Bool, "imu_init", 10)							# QoS：reliable
		self.publisher = self.create_publisher(MotorSpeedMessage, "wheel_motor_speeds", qos_profile)	# QoS：best-effort

		# サブスクライバの生成
		self.dev_status_subscription = self.create_subscription(
			SpikeDevStatusMessage, "spike_device_status", self.dev_status_on_subscribe, qos_profile)
		self.button_status_subscription = self.create_subscription(
			ButtonStatusMessage, "spike_button_status", self.button_status_on_subscribe, qos_profile)
		
		self.timer_callback = self.create_timer(0.001, self.drive)

	def drive(self):	# 1ms周期で実行

		servo = 0
		esc = 0

		if self.drive_is_first == True:
			
			print("drive start")
			self.fpv.open()									# 色の定義とカメラウィンドウの設定
			q_cam.put(True)									# mode='play'なら関数play中のq_cam.getに、mode='drive'なら関数camera中のq_cam.getに飛ぶ
			self.t1 = time.time()
			self.drive_is_first = False

		if self.is_end == False:
			self.t2 = time.time() - self.t1 		# タイムスタンプ
			self.t1 = time.time()

			frame = q_img.get()						# キューに入っているアイテム「frame」を取り除いて変数frameに代入する

			# 画像の切り取り(第一引数：縦方向の切り取り、第二引数：横方向の切り取り)
			frame = frame[0:80, 50:110]	

			self.t3 = time.time()
			if v_auto_on.value: 					# AI走行ON
				servo, esc = self.model_a.predict(frame)
						
			else:  									# 手動走行
				servo = v_wheel.value 
				esc = v_trigger.value 				# スティックでの操作量(servoは横、escは縦)
			self.t3 = time.time() - self.t3

			v_servo.value = limit(servo + v_servo_trim.value) 	# 横の操作量(十字ボタンの横で横の操作量を軽減できる)
			if v_esc_on.value:
				v_esc.value = limit(self.speed.trim(esc)) 			# 縦の操作量(十字ボタン縦でスピードの速さを変えられる)
				v_servo.value = limit(self. speed.trim_s(servo))
			else:
				v_esc.value = 0

			if v_log_on.value: 											# logの出力
				v_num.value = self.log.append(self.t1, self.t2, self.t3, frame, servo, esc) # 返り値はデータの数

			if count_reset.value:										# エンコーダ値のリセット（R1ボタン）
				self.r_init = 0
				self.l_init = 0
				self.model_mode = 0
				count_reset.value = not count_reset.value

			if not self.fpv.disp(frame, servo, esc): 						# ディスプレイを表示．表示されていなければ（返り値がFalseであれば）駆動しない
				v_ok.value = False
				self.is_end = True
				
			# センターボタンが押されたらESCと自動走行のON・OFFを同時に切り替える (AUTOが基準)
			if (self.rev_button_val & 0b00010000) and not (self.pre_button_val & 0b00010000):
				if v_auto_on.value:
					v_auto_on.value = False
					v_esc_on.value = False
				else:
					v_auto_on.value = True
					v_esc_on.value = True
				print('ESC', 'ON' if v_esc_on.value else 'OFF')
				print('AUTO', 'ON' if v_auto_on.value else 'OFF')
			self.pre_button_val = self.rev_button_val			

			# モータ指令値を計算
			pulse_L = v_esc.value * (1 - v_servo.value * 0.8)
			pulse_R = v_esc.value * (1 + v_servo.value * 0.8)

			if pulse_L > 1:
				pulse_L = 1
			if pulse_L < -1:
				pulse_L = -1

			if pulse_R > 1:
				pulse_R = 1
			if pulse_R < -1:
				pulse_R = -1

			l_motor_speed = round(cfg.SPEER_MAX * pulse_R)
			r_motor_speed = round(cfg.SPEER_MAX * pulse_L)

			motor_speed = MotorSpeedMessage()
			motor_speed.right_motor_speed = r_motor_speed
			motor_speed.left_motor_speed = l_motor_speed
			motor_speed.arm_motor_speed = 0

			# モータ指令値のパブリッシュ
			self.publisher.publish(motor_speed)

		#角度が変化したら表示
		if v_angle_log_on.value:
			if self.rev_x_angle != self.pre_rev_x_angle or self.rev_z_angle != self.pre_rev_z_angle:
				self.get_logger().info("---")
				self.get_logger().info("x_angle : " + str(self.rev_x_angle))
				self.get_logger().info("z_angle : " + str(self.rev_z_angle))
		self.pre_rev_x_angle = self.rev_x_angle
		self.pre_rev_z_angle = self.rev_z_angle

		# 右ボタンが押されたら角度をリセット
		if (self.rev_button_val & 0b00000010) and not (self.pre_button_val & 0b00000010):
			imu_init_msg = Bool()
			imu_init_msg.data = True
			self.imu_init_publisher.publish(imu_init_msg)	# リセット指令をパブリッシュ
			self.get_logger().info("")
			self.get_logger().info("reset angle")
			self.get_logger().info("")
		self.pre_button_val = self.rev_button_val			

		if (self.is_end == True) and (self.end_count == 0):  	
			self.log.close()
			self.fpv.close()
			v_ok.value = False
			self.end_count = 1
			print('drive end')
	# drive() end #

	# センサ値等のサブスクライバーコールバック	
	def dev_status_on_subscribe(self, devise_status):

		self.rev_color_mode = devise_status.color_mode_id
		if self.rev_color_mode == 1:
			self.rev_color_sensor_ambient = devise_status.color_sensor_value_1
		elif self.rev_color_mode == 2:
			self.rev_color_sensor_color = devise_status.color_sensor_value_1
		elif self.rev_color_mode == 3:
			self.rev_color_sensor_refrection = devise_status.color_sensor_value_1
		elif self.rev_color_mode == 4:
			self.rev_color_sensor_r = devise_status.color_sensor_value_1
			self.rev_color_sensor_g = devise_status.color_sensor_value_2
			self.rev_color_sensor_b = devise_status.color_sensor_value_3

		self.rev_ultrasonic_mode = devise_status.ultrasonic_mode_id
		self.rev_ultrasonic_val = devise_status.ultrasonic_sensor
		self.rev_arm_cnt = devise_status.arm_count
		self.rev_right_cnt = devise_status.right_count
		self.rev_left_cnt = devise_status.left_count
		self.rev_x_angle = devise_status.x_angle
		self.rev_z_angle = devise_status.z_angle
		'''
		print("right_count : " + str(self.rev_right_cnt) + " left_count : " + str(self.rev_left_cnt))
		print("gyro : " + str(self.rev_x_ang_vel))
		'''
		if self.r_init == 0:					#エンコーダ値のリセット
			self.r_init = self.rev_right_cnt
			self.l_init = self.rev_left_cnt
		
		self.r_current = self.rev_right_cnt - self.r_init
		self.l_current = self.rev_left_cnt - self.l_init

		if v_log_print_on.value:
			print("model_mode : " + str(self.model_mode) + " right_count : " + str(self.r_current) + " left_count : " + str(self.l_current))

	# ボタン押下情報のサブスクライバコールバック
	def button_status_on_subscribe(self, button_status):
		self.rev_button_val = button_status.button		


def limit(value):				#value <= |1.0|にする関数
	value = min(value, 1.0)
	value = max(-1.0, value)
	return value

def camera(fps, width, height, rotate): #カメラの設定
	print("camera start")

	capture0 = cv2.VideoCapture(0)
	#縦横の長さ、FPSの設定
	capture0.set(cv2.CAP_PROP_FRAME_WIDTH, width)
	capture0.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
	capture0.set(cv2.CAP_PROP_FPS, fps)

	print('width:', capture0.get(cv2.CAP_PROP_FRAME_WIDTH), '(', width, ')')
	print('height:', capture0.get(cv2.CAP_PROP_FRAME_HEIGHT), '(', height, ')')
	print('fps:', capture0.get(cv2.CAP_PROP_FPS), '(', fps, ')')

	v_fps.value = fps

	q_cam.get()

	while v_ok.value:
		ret, frame = capture0.read() #retは画像の取得が成功したかどうかのT/Fが入り、frameに画像データを入れる
		if rotate:
			frame = cv2.rotate(frame, cv2.ROTATE_180)
		q_img.put_nowait(frame)		 #q_imgに画像データをitemとして入れる

	print('camera end')

def play(log_dir):
	print("play start")

	f = open(log_dir + '/log.csv','r')
	reader = csv.DictReader(f)

	q_cam.get()
	v_esc_on.value = True

	for row in reader:
		if not v_ok.value:
			break
		index = int(row['index'])
		print(index)
		time2 = float(row['time2'])
		cam_file = row['jpg']
		v_wheel.value = float(row['servo'])
		v_trigger.value = float(row['esc'])

		frame = cv2.imread(log_dir + '/' + cam_file, cv2.IMREAD_UNCHANGED)
		q_img.put(frame)
		time.sleep(time2)

	f.close()
	print('play end')
	v_ok.value = False
	q_img.put(frame)

class Fpv: #ディスプレイ関係
	def open(self):
		self.BLACK   = (  0,  0,  0) #色の定義
		self.RED	 = (  0,  0,255)
		self.GREEN   = (  0,255,0  )
		self.YELLOW  = (  0,255,255)
		self.BLUE	= (255,  0,  0)
		self.MAGENTA = (255,  0,255)
		self.CYAN	= (255,255,  0)
		self.WHITE   = (255,255,255)

		window_style = cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL	#画像の大きさを自分で調整できるオプションを設定する
		cv2.namedWindow('Camera', window_style)

	def disp(self, frame, servo, esc):
		if not v_esc_on.value: #現在の状態によって直線の色を変更
			color = self.WHITE
		elif v_auto_on.value and v_log_on.value:
			color = self.MAGENTA
		elif v_auto_on.value:
			color = self.YELLOW
		elif v_log_on.value:
			color = self.RED
		else:
			color = self.GREEN

		height, width, _ = frame.shape #ディスプレイウィンドウの大きさ設定
		x0=int(round(width/2))
		y0=int(round(height))
		x1=int(round(width/2 + width * servo / 2))
		y1=int(round(height - height * esc))
		cv2.line(frame,(x0 ,y0),(x1,y1),color,2) #直線の描画
		cv2.imshow('Camera', frame) #第一引数はWindowの名前、第二引数は表示したい画像（今回は読み込んだ画像）

		wk = cv2.waitKey(1) & 0xFF  #Cameraウィンドウの操作
		if wk == ord('q'):			#プログラムの終了
			return False
		elif wk == ord('w'):
			v_esc_on.value = not v_esc_on.value
			print('ESC', 'ON' if v_esc_on.value else 'OFF')
		elif wk == ord('e'):
			v_auto_on.value = not v_auto_on.value
			print('AUTO', 'ON' if v_auto_on.value else 'OFF')
		elif wk == ord('r'):
			v_log_on.value = not v_log_on.value
			print('LOG', 'ON' if v_log_on.value else 'OFF')
		elif wk == ord('t'):
			v_log_print_on.value = not v_log_print_on.value
			print('LOG_PRINT', 'ON' if v_log_print_on.value else 'OFF')
		return True

	def close(self):
		cv2.destroyAllWindows()

class Log: #ログの出力関係
	def __init__(self, log_dir):
		self.log_dir = log_dir

		print(self.log_dir)
		#os.makedirs(self.log_dir)
		#os.makedirs(self.log_dir + 'jpg')
		if not os.path.isdir(self.log_dir):
			os.makedirs(self.log_dir)
			print(self.log_dir)
		if not os.path.isdir(self.log_dir + 'jpg'):
			os.makedirs(self.log_dir + 'jpg')
			print(self.log_dir + 'jpg')
		#行列の初期化
		self.n = 0
		self.log_index = list()
		self.log_time1 = list()
		self.log_time2 = list()
		self.log_time3 = list()
		self.log_jpg = list()
		self.log_servo = list()
		self.log_esc = list()

	def append(self, t1, t2, t3, frame, servo, esc): #行列に値を入れる
		self.n += 1
		jpg_file_name = 'jpg/cam_{:05d}.jpg'.format(self.n)
		cv2.imwrite(self.log_dir + jpg_file_name, frame, [cv2.IMWRITE_JPEG_QUALITY, 50])

		self.log_index.append(self.n)
		self.log_time1.append(t1)
		self.log_time2.append(t2)
		self.log_time3.append(t3)
		self.log_jpg.append(jpg_file_name)
		self.log_servo.append(servo)
		self.log_esc.append(esc)

		if self.n % 100 == 0:					#値の数が100ごとにログを出力
			print('log count:', self.n)
		return self.n

	def close(self):							#行列に入れた値をcsvファイルに 再帰的に出力
		
		#print(self.log_index)
		#print(self.log_dir)
		if self.n > 0:
			f = open(self.log_dir + 'log.csv', 'w')
			print(f)
			f.write( \
				'index,' + \
				'time1,' + \
				'time2,' + \
				'time3,' + \
				'jpg,' + \
				'servo,' + \
				'esc,' + \
				'\n')
			for i in range(self.n):
				#print(i)
				f.write(str(self.log_index[i]) + ',')
				f.write(str(self.log_time1[i]) + ',')
				f.write(str(self.log_time2[i]) + ',')
				f.write(str(self.log_time3[i]) + ',')
				f.write(self.log_jpg[i] + ',')
				f.write(str(self.log_servo[i]) + ',')
				f.write(str(self.log_esc[i]) + ',')
				f.write('\n')
			f.close()
			print('log.csv saved', self.n)

class Model: #モデルファイル関係
	def __init__(self, model_path): #モデルのロード
		self.model_path = model_path
		if os.path.exists(model_path):
			print('model file', model_path, 'loading')
			self.interpreter = tf.lite.Interpreter(model_path)
			self.interpreter.allocate_tensors()
			self.input_details = self.interpreter.get_input_details()
			self.output_details = self.interpreter.get_output_details()
			print('model load OK')
			v_model.value = True
		else:
			print('model file', model_path, 'is not exists')
			v_model.value = False

	def predict(self, frame): #読み込んだモデルから操作量を予測する
		if v_model.value:
			x = frame
			x = x.astype('float32') / 255
			x = np.expand_dims(x, axis=0)

			self.interpreter.set_tensor(self.input_details[0]['index'], x)
			self.interpreter.invoke()

			output_data0 = self.interpreter.get_tensor(self.output_details[0]['index'])
			output_data1 = self.interpreter.get_tensor(self.output_details[1]['index'])

			servo = output_data0[0][0]
			esc = output_data1[0][0]
			return servo, esc
		else:
			if v_auto_on.value:
				print('AUTO OFF', self.model_path, 'is none')
				v_auto_on.value = False
			return 0, 0

class Speed:
	def __init__(self, speed_trim, speed_trim_ai): #スピードトリムの読み込み
		v_speed_trim.value = speed_trim
		v_speed_trim_ai.value = speed_trim_ai

	def trim(self, esc):
		esc *= v_speed_trim.value

		if v_auto_on.value:
			if 0.2 < v_trigger.value or v_trigger.value < -0.2: #AI操作の最中に右スティック操作量が入力されると手動操縦される
				#手動操縦
				esc = v_trigger.value
			else:
				esc *= v_speed_trim_ai.value
		return esc

	def trim_s(self, servo):
		if v_auto_on.value:
			if 0.2 < v_wheel.value or v_wheel.value < -0.2:
				servo = v_wheel.value 
			
		return servo

def main(args=None):

	print("main start")

	#fpv = Fpv()												#fpvクラスの読み込み
	#log = Log(cfg.LOG_DIR)									#ログ出力のセットアップ
	#model = Model(cfg.MODEL_FILE)							#モデルがあるかの確認
	#speed = Speed(cfg.SPEED_TRIM, cfg.SPEED_TRIM_AI)		#スピードトリムの読み込み

	executor_t = concurrent.futures.ThreadPoolExecutor(max_workers=13)		#最大でmax_wokrer個のスレッドを非同期実行に使う

	mode = 'drive'
	if len(sys.argv) == 2:									#コマンドライン引数（playの指定があるときはplayに飛ぶ）
		mode = sys.argv[1] 

	if mode == 'play': 
		executor_t.submit(play, cfg.LOG_DIR)
		#executor_t.submit(drive, fpv, log, model, speed)
	
	elif mode == 'drive':
		executor_t.submit(camera, cfg.CAMERA_FPS, cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT, cfg.CAM_ROTATE) #カメラのセットアップ
		#executor_t.submit(drive, fpv, log, model, speed) #操舵量の計算

		from ros2_ai_driver.dualshock4 import dualshock4 as dualshock4
		executor_t.submit(dualshock4, v_ok,  #各種ボタンの設定(詳細はdualshock4.pyにて)
			v_wheel, v_trigger,
			v_esc_on, v_auto_on, v_log_on,
			v_log_print_on, v_angle_log_on, count_reset, v_servo_trim, v_speed_trim)

		#from ros2_ai_driver.raspimouse import raspimouse as raspimouse
		#executor_t.submit(raspimouse, v_ok, v_servo, v_esc, v_fps, cfg.SPEER_MAX) #ラズパイマウスを動作させる
	# RCLの初期化
	rclpy.init(args=args)

	# ノードの生成(インスタンスの生成？)
	node = AIdriveNode()

	# ノード終了まで待機
	rclpy.spin(node)

	# ノードの破棄
	node.destroy_node()

	# RCLのシャットダウン
	rclpy.shutdown()

if __name__ == "__main__":
	main()
