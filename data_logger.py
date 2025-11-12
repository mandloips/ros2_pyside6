# data_logger.py
import csv
import os
from datetime import datetime

# Define the file path where the data will be logged
LOG_FILE = "pose_data_log.csv"

def init_csv():
    """Initialize the CSV file with headers if it does not exist"""
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write the header row
            writer.writerow(["Timestamp", "X", "Y", "Theta", "Linear Velocity", "Angular Velocity"])

def log_pose_data(x, y, theta, linear_velocity, angular_velocity):
    """Log the pose data to a CSV file"""
    # Ensure the CSV file is initialized
    init_csv()
    
    # Get the current timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    # Write the data to the CSV file
    with open(LOG_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, x, y, theta, linear_velocity, angular_velocity])

    # print(f"Logged data: {timestamp}, {x}, {y}, {theta}, {linear_velocity}, {angular_velocity}")