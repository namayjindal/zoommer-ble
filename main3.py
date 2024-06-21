import sys
import os
import csv
import asyncio
import json
from datetime import datetime
from bleak import BleakScanner, BleakClient
from PyQt5.QtWidgets import (QApplication, QWizard, QWizardPage, QLabel, QLineEdit, QVBoxLayout, QDateEdit, QPushButton, QComboBox, QMessageBox, QInputDialog)
from PyQt5.QtCore import QDate, QThread, pyqtSignal, QTimer, QObject
import hashlib
import time
import string
import random

# Load exercise configuration from a JSON file
EXERCISE_CONFIG = {
    "Dribbling in Figure 8": {
      "sensors": [1, 2, 3, 4, 5],
      "columns": [
        "timestamp",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z",
        "ball_Accel_X",
        "ball_Accel_Y",
        "ball_Accel_Z",
        "ball_Gyro_X",
        "ball_Gyro_Y",
        "ball_Gyro_Z"
      ]
    },
    "Dribbling in Figure O": {
      "sensors": [1, 2, 3, 4, 5],
      "columns": [
        "timestamp",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z",
        "ball_Accel_X",
        "ball_Accel_Y",
        "ball_Accel_Z",
        "ball_Gyro_X",
        "ball_Gyro_Y",
        "ball_Gyro_Z"
      ]
    },
    "Large Ball Bounce and Catch": {
      "sensors": [1, 2, 5],
      "columns": [
        "timestamp",
        "right_hand_index",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_index",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z",
        "ball_index",
        "ball_Accel_X",
        "ball_Accel_Y",
        "ball_Accel_Z",
        "ball_Gyro_X",
        "ball_Gyro_Y",
        "ball_Gyro_Z"
      ]
    },
    "Hit Balloon Up": {
      "sensors": [1, 2],
      "columns": [
        "timestamp",
        "right_hand_index",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_index",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z"
      ]
    },
    "Jumping Jack with Clap": {
      "sensors": [1, 2, 3, 4],
      "columns": [
        "timestamp",
        "right_hand_index",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_index",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z",
        "right_leg_index",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_index",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Forward Backward Spread Legs and Back": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Criss Cross with Clapping": {
      "sensors": [1, 2, 3, 4],
      "columns": [
        "timestamp",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Criss Cross without Clapping": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Criss Cross (leg forward)": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Alternate Feet Forward Backward": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Skipping": {
      "sensors": [1, 2, 3, 4],
      "columns": [
        "timestamp",
        "right_hand_Accel_X",
        "right_hand_Accel_Y",
        "right_hand_Accel_Z",
        "right_hand_Gyro_X",
        "right_hand_Gyro_Y",
        "right_hand_Gyro_Z",
        "left_hand_Accel_X",
        "left_hand_Accel_Y",
        "left_hand_Accel_Z",
        "left_hand_Gyro_X",
        "left_hand_Gyro_Y",
        "left_hand_Gyro_Z",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Jumping Jack without Hands": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Jump with feet symmetrically": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Jump with feet asymmetrically": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Hop between lines": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Hopping forward one one leg": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Step down from height": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Step over an obstacle": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    },
    "Stand on one leg": {
      "sensors": [3, 4],
      "columns": [
        "timestamp",
        "right_leg_Accel_X",
        "right_leg_Accel_Y",
        "right_leg_Accel_Z",
        "right_leg_Gyro_X",
        "right_leg_Gyro_Y",
        "right_leg_Gyro_Z",
        "left_leg_Accel_X",
        "left_leg_Accel_Y",
        "left_leg_Accel_Z",
        "left_leg_Gyro_X",
        "left_leg_Gyro_Y",
        "left_leg_Gyro_Z"
      ]
    }
  }

# UUIDs and other data
UART_SERVICE_UUIDS = [
    ("Sense Right Hand", "8E400004-B5A3-F393-E0A9-E50E24DCCA9E", "8E400006-B5A3-F393-E0A9-E50E24DCCA9E"),
    ("Sense Left Hand", "6E400001-B5A3-F393-E0A9-E50E24DCCA9E", "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    ("Sense Right Leg", "7E400001-A5B3-C393-D0E9-F50E24DCCA9E", "7E400003-A5B3-C393-D0E9-F50E24DCCA9E"),
    ("Sense Left Leg", "6E400001-B5C3-D393-A0F9-E50F24DCCA9E", "6E400003-B5C3-D393-A0F9-E50F24DCCA9E"),
    ("Sense Ball", "9E400001-C5C3-E393-B0A9-E50E24DCCA9E", "9E400003-C5C3-E393-B0A9-E50E24DCCA9E"),
]
buffers = {i: "" for i in range(1, 6)}
start_times = {i: None for i in range(1, 6)}
sensor_data = {i: {"timestamp": None, "values": [None] * 6} for i in range(1, 6)}

csv_filename = ""
STOP_FLAG = False
error_counter = 0
MAX_ERRORS = 4

def generate_hashed_id(info):
    # Generate a random string
    random_str = ''.join(random.choices(string.ascii_letters + string.digits, k=8))

    # Get the current timestamp
    timestamp = str(time.time())

    # Combine the information with the random string and timestamp
    input_str = f"{info}_{random_str}_{timestamp}"

    # Generate the hash
    hash_object = hashlib.sha256(input_str.encode('utf-8'))
    return hash_object.hexdigest()[:20]  # Use the first 20 characters of the hash

# Function to append the new record to the exercise JSON file
def append_to_exercise_record(date, record):
    filename = f'exercise_records_{date}.json'
    try:
        with open(filename, 'r') as f:
            records = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        records = []

    records.append(record)
    with open(filename, 'w') as f:
        json.dump(records, f, indent=4)

# Define global variable for selected exercise configuration
selected_exercise_config = None

class GuiUpdater(QObject):
    showMessageSignal = pyqtSignal(str)
    stopExerciseSignal = pyqtSignal()
    stopForErrorsSignal = pyqtSignal(str)

    def __init__(self):
        super().__init__()

    def show_message(self, message):
        msgBox = QMessageBox()
        msgBox.setText(message)
        msgBox.exec()

    def stop_exercise(self):
        ex.stopExercise()

gui_updater = GuiUpdater()
gui_updater.showMessageSignal.connect(gui_updater.show_message)
gui_updater.stopExerciseSignal.connect(gui_updater.stop_exercise)
gui_updater.stopForErrorsSignal.connect(gui_updater.show_message)

async def notification_handler(sender, data, sensor_id):
    global buffers, start_times, STOP_FLAG, error_counter, selected_exercise_config
    if STOP_FLAG:
        return
    if start_times[sensor_id] is None:
        start_times[sensor_id] = datetime.now()
    buffers[sensor_id] += data.decode('utf-8')
    buffer = buffers[sensor_id]
    while '\n' in buffer:
        line, buffer = buffer.split('\n', 1)
        buffers[sensor_id] = buffer
        if line.strip() == "":
            continue
        try:
            parts = line.split(',')
            if len(parts) != 7:
                raise ValueError(f"Incorrect number of values: {len(parts)}. Received line: {line}")
            imu_values = list(map(float, parts))
            elapsed_time = (datetime.now() - start_times[sensor_id]).total_seconds() * 1000
            sensor_data[sensor_id]["timestamp"] = elapsed_time
            sensor_data[sensor_id]["values"] = imu_values
            if all(sensor_data[i]["timestamp"] is not None and sensor_data[i]["values"][0] is not None for i in selected_exercise_config["sensors"]):
                timestamp = round(sensor_data[selected_exercise_config["sensors"][0]]["timestamp"], 3)
                row = [timestamp] + sum((sensor_data[i]["values"] for i in selected_exercise_config["sensors"]), [])
                if len(row) != len(selected_exercise_config["columns"]):
                    raise ValueError("Row has an incorrect number of values")
                with open(csv_filename, 'a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(row)
                for i in selected_exercise_config["sensors"]:
                    sensor_data[i]["timestamp"] = None
                    sensor_data[i]["values"] = [None] * 7
        except ValueError as e:
            error_counter += 1
            print(f"Error: {e}. Received line: {line}")
            if error_counter >= MAX_ERRORS:
                QTimer.singleShot(0, gui_updater.stopExerciseSignal.emit)
                QTimer.singleShot(0, lambda: gui_updater.stopForErrorsSignal.emit("Bad data, stop and restart"))

async def connect_to_sensor(device, sensor_id, char_uuid):
    async with BleakClient(device) as client:
        if client.is_connected:
            print(f"Connected to {device.name}")
            await client.start_notify(char_uuid, lambda sender, data: asyncio.create_task(notification_handler(sender, data, sensor_id)))
            while not STOP_FLAG:
                await asyncio.sleep(0.1)

async def scan_and_connect():
    tasks = []
    devices = await BleakScanner.discover()
    connected_sensors = []
    for sensor in selected_exercise_config["sensors"]:
        name, service_uuid, char_uuid = UART_SERVICE_UUIDS[sensor-1]
        for device in devices:
            if device.name == name:
                tasks.append(connect_to_sensor(device, sensor, char_uuid))
                connected_sensors.append(name)
                break
    gui_updater.showMessageSignal.emit(f"Connected to: {', '.join(connected_sensors)}")
    await asyncio.gather(*tasks)


class AsyncRunner(QThread):
    updateStatus = pyqtSignal(str)
    sensorsConnected = pyqtSignal()

    async def scan_and_connect(self):
        await scan_and_connect()
        self.sensorsConnected.emit()
        self.updateStatus.emit("Sensors connected")

    def run(self):
        global STOP_FLAG
        STOP_FLAG = False
        self.updateStatus.emit("Connecting to sensors...")
        asyncio.run(self.scan_and_connect())

    def stop(self):
        global STOP_FLAG
        STOP_FLAG = True

class StartPage(QWizardPage):
    def __init__(self, parent=None):
        super(StartPage, self).__init__(parent)
        self.setTitle("Start Page")
        layout = QVBoxLayout()
        #self.setFixedSize(500, 400)
        self.school_name_input = QLineEdit()
        self.date_input = QDateEdit()
        self.date_input.setCalendarPopup(True)
        self.date_input.setDisplayFormat('dd/MM/yyyy')
        self.date_input.setDate(QDate.currentDate())
        layout.addWidget(QLabel("School Name:"))
        layout.addWidget(self.school_name_input)
        layout.addWidget(QLabel("Date:"))
        layout.addWidget(self.date_input)
        self.setLayout(layout)

    def initializePage(self):
        self.school_name_input.setText(get_saved_school_name())
        self.date_input.setDate(get_saved_date())

    def validatePage(self):
        save_school_name(self.school_name_input.text())
        save_date(self.date_input.date())
        return True

def save_school_name(name):
    with open('school_name.txt', 'w') as f:
        f.write(name)

def get_saved_school_name():
    try:
        with open('school_name.txt', 'r') as f:
            return f.read().strip()
    except FileNotFoundError:
        return ""

def save_date(date):
    with open('date.txt', 'w') as f:
        f.write(date.toString("yyyyMMdd"))

def get_saved_date():
    try:
        with open('date.txt', 'r') as f:
            date_str = f.read().strip()
            return QDate.fromString(date_str, "yyyyMMdd")
    except FileNotFoundError:
        return QDate.currentDate()

class MainPage(QWizardPage):
    def __init__(self, parent=None):
        super(MainPage, self).__init__(parent)
        self.setTitle("Main Page")
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()
        self.setFixedSize(500, 500)

        self.name_label = QLabel("Name:")
        self.name_input = QLineEdit()
        self.layout.addWidget(self.name_label)
        self.layout.addWidget(self.name_input)

        self.grade_label = QLabel("Grade:")
        self.grade_input = QLineEdit()
        self.layout.addWidget(self.grade_label)
        self.layout.addWidget(self.grade_input)

        self.height_label = QLabel("Height:")
        self.height_input = QLineEdit()
        self.layout.addWidget(self.height_label)
        self.layout.addWidget(self.height_input)

        self.gender_label = QLabel("Gender:")
        self.gender_dropdown = QComboBox()
        self.gender_dropdown.addItems(["Male", "Female"])
        self.layout.addWidget(self.gender_label)
        self.layout.addWidget(self.gender_dropdown)

        self.exercise_name_label = QLabel("Exercise Name:")
        self.exercise_name_dropdown = QComboBox()
        self.exercise_name_dropdown.addItems(list(EXERCISE_CONFIG.keys()))
        self.layout.addWidget(self.exercise_name_label)
        self.layout.addWidget(self.exercise_name_dropdown)

        self.status_label = QLabel("")
        self.layout.addWidget(self.status_label)
        self.timer_label = QLabel("Elapsed Time: 0s")
        self.layout.addWidget(self.timer_label)
        self.start_button = QPushButton('Start Exercise', self)
        self.start_button.clicked.connect(self.startExercise)
        self.layout.addWidget(self.start_button)
        self.stop_button = QPushButton('Stop Exercise', self)
        self.stop_button.clicked.connect(self.stopExercise)
        self.stop_button.setEnabled(False)
        self.layout.addWidget(self.stop_button)
        self.setLayout(self.layout)
        self.timer = QTimer(self)
        self.elapsed_time = 0
        self.timer.timeout.connect(self.update_timer)
        self.async_runner = AsyncRunner()
        self.async_runner.updateStatus.connect(self.setStatus)
        self.async_runner.sensorsConnected.connect(self.start_timer)

    def toggle_timer_label(self, show):
        self.timer_label.setVisible(show)

    def setStatus(self, status):
        self.status_label.setText(status)

    def startExercise(self):
        global csv_filename, start_times, selected_exercise_config

        exercise_name = self.exercise_name_dropdown.currentText()
        selected_exercise_config = EXERCISE_CONFIG[exercise_name]
        start_times = {i: None for i in selected_exercise_config["sensors"]}

        self.start_timer()
        self.toggle_timer_label(True)

        school_name = get_saved_school_name()
        date_selected = get_saved_date().toString("ddMMyyyy")
        grade = self.grade_input.text()

        # Generate a unique hashed ID for the CSV file
        hash_info = f"{school_name}_{date_selected}_{grade}_{exercise_name}"
        hashed_id = generate_hashed_id(hash_info)

        os.makedirs("./data", exist_ok=True)
        csv_filename = f"./data/{hashed_id}.csv"
        with open(csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(selected_exercise_config["columns"])

        # Prepare record to later append to the exercise log
        global exercise_record
        exercise_record = {
            "school_name": school_name,
            "date": date_selected,
            "name": self.name_input.text(),
            "grade": grade,
            "height": self.height_input.text(),
            "gender": self.gender_dropdown.currentText(),
            "exercise_name": exercise_name,
            "file_id": hashed_id,
            "label": None  # Initially, label is None
        }

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.async_runner.start()

    def stopExercise(self):
        self.async_runner.stop()
        self.async_runner.wait()
        self.timer.stop()  # Ensure the timer stops here
        exercise_name = self.exercise_name_dropdown.currentText()

        msgBox = QMessageBox(self)
        msgBox.setIcon(QMessageBox.Question)
        msgBox.setText("Do you want to keep the data?")
        msgBox.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        yesButton = msgBox.button(QMessageBox.Yes)
        msgBox.setDefaultButton(yesButton)
        retval = msgBox.exec()

        if retval == QMessageBox.Yes:
            label, ok = QInputDialog.getItem(
                self, 'Input Dialog', 'Enter a label for the data:', ["Good", "Idle", "Anomaly"], 0, False
            )
            if ok:
                global csv_filename, exercise_record
                exercise_record["label"] = label  # Update the label in the record
                base, ext = os.path.splitext(csv_filename)
                new_filename = f"{base}_{exercise_name}{ext}"
                os.rename(csv_filename, new_filename)

                # Append the record with the label to the exercise log
                append_to_exercise_record(exercise_record["date"], exercise_record)

                self.setStatus(f"Data labeled as {label} and saved to {new_filename}")
            else:
                self.setStatus("Label input canceled")
        else:
            os.remove(csv_filename)
            self.setStatus("Data discarded")

        self.elapsed_time = 0
        self.timer_label.setText("Elapsed Time: 0s")
        self.toggle_timer_label(False)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.timer.stop()  # Ensure the timer stops here too

    def start_timer(self):
        self.elapsed_time = 0
        self.timer.start(1000)
        self.setStatus("Connected to sensors. Tracking exercises now.")

    def update_timer(self):
        self.elapsed_time += 1
        self.timer_label.setText(f"Elapsed Time: {self.elapsed_time}s")
        if self.elapsed_time >= 6:
            self.setStatus("Tracking exercises now...")


class ExerciseApp(QWizard):
    def __init__(self):
        super().__init__()
        self.addPage(StartPage())
        self.addPage(MainPage())
        self.addPage(FinishPage())
        self.setWindowTitle("Exercise App")
    
    def stopExercise(self):
        self.findChild(MainPage).stopExercise()

class FinishPage(QWizardPage):
    def __init__(self, parent=None):
        super(FinishPage, self).__init__(parent)
        self.setTitle("Finish Page")
        layout = QVBoxLayout()
        self.setFixedSize(500, 400)
        self.finish_label = QLabel("Exercise data collection finished!")
        layout.addWidget(self.finish_label)
        self.setLayout(layout)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = ExerciseApp()
    ex.show()
    sys.exit(app.exec_())