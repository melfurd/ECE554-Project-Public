import requests
import time
import RPi.GPIO as GPIO
import sys
import signal
import datetime
import threading
import os
import email, smtplib, ssl
import shutil
import csv
import board
import busio
import adafruit_mpl115a2
import re 
import imageio
from scipy.misc import imread
from scipy.linalg import norm
from scipy import sum, average
from gpiozero import Buzzer
from picamera import PiCamera
from gpiozero import MotionSensor as motion
from threading import Thread
from flask import Flask, render_template
from pad4pi import rpi_gpio
from email import encoders
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from queue import Queue

#get working directory
current_working_directory = os.getcwd()

#Semaphore
keep_running_app_semaphore = True
camera_in_use_semaphore = False
save_next_five_picture_semaphore = False

#Queue for last 10 pictures taken
saved_images_queue = Queue()

#image files
file_name_second_last_taken_picture = ""
file_name_last_taken_picture = ""
file_name_last_analyzed = "None"

#PIR
pir = motion(4)

#Buzzer
buzzer = Buzzer(17)

#Flask App
app = Flask(__name__)

#email counter
email_locking_semaphore = False
enable_emails = False

#picture as sensor
enable_picture_check_as_sensor = False

# values for keypress
#incorrect keypress counters
keypress_incorrect_counter = 0
keypress_thirty_second_cooldown = 30
keypress_in_last_thirty_seconds = False
new_string = ""

#keypad
#https://www.instructables.com/id/Using-a-keypad-with-Raspberry-Pi/
keypad = [[1, 2, 3],[4, 5, 6],[7, 8, 9],["*", 0, "#"]]
#output pins for the keypad and breadboard
row_pin = [26, 19, 13, 6]
column_pin = [5, 21, 20]
#code to create the keypad connections
factory = rpi_gpio.KeypadFactory()
keypad = factory.create_keypad(keypad=keypad, row_pins=row_pin, col_pins=column_pin)

#keypad mastercodes
import csv
with open('code.csv', 'r') as f:
    reader = csv.reader(f)
    codes = list(reader)

#LED Lights
ledred_pin = 24
ledblue_pin = 23
GPIO.setup(ledred_pin, GPIO.OUT)
GPIO.setup(ledblue_pin, GPIO.OUT)

#email to txt
thefile = open("emails.txt", "r")
sendToEmail = str(thefile.read())

#Turn on and off lights
def disarm_alarm_lights():
    GPIO.output(ledred_pin, False)
    GPIO.output(ledblue_pin, True)

def arm_alarm_lights():
    GPIO.output(ledred_pin, True)
    GPIO.output(ledblue_pin, False)

#Log Event in File
def save_text_in_csv_log(action, text_to_save):
    with open('Log.csv', 'a') as f:
        time_string = str("%.20f" % time.time())
        f.write(time.strftime("%Y%m%d-%H%M%S") + ', ' + time_string + ', ' + action + ', ' + text_to_save + '\n')

#Returns status of PIR sensor, motion is detected
def is_motion_detected():

    timeStart = float("%.20f" % time.time())
    value = pir.motion_detected
    timeEnd = float("%.20f" % time.time())
    save_text_in_csv_log("PIR Sensor Time", str(timeEnd-timeStart))
    #if not (change_in_variable and value):
    #    global change_in_variable
    #    change_in_variable = value
        
    if value:
        save_text_in_csv_log("PIR Sensor " + str(value), str(timeEnd))
        #print("PIR = 1")
    return value

#Returns status of the door if it is open or not
def is_door_open():
    timeStart = float("%.20f" % time.time())
    GPIO.setmode(GPIO.BCM) 
    DOOR_SENSOR_PIN = 27
    GPIO.setup(DOOR_SENSOR_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    is_door_open = GPIO.input(DOOR_SENSOR_PIN)
    timeEnd = float("%.20f" % time.time())
    save_text_in_csv_log("Door Sensor Time", str(timeEnd-timeStart))
    if is_door_open == 1:
        #print(is_door_open)
        return True
    else:
        return False

#Pressure and Temperature Sensor
#https://github.com/adafruit/Adafruit_CircuitPython_MPL115A2.git
#use library provided
#in try and catch because we don't have a sodering iron to soder it
def check_pressure_and_temperature_sensor():
    timeStart = float("%.20f" % time.time())
    try:
        
        i2c = busio.I2C(board.SCL, board.SDA)
        mpl = adafruit_mpl115a2.MPL115A2(i2c)
        print(mpl.temperature)
        timeEnd = float("%.20f" % time.time())
        if int(mpl.temperature) > 25:
            save_text_in_csv_log("Temperature Above", str(timeEnd-timeStart))
            return True
        elif int(mpl.temperature) < 15:
            save_text_in_csv_log("Temperature Below", str(timeEnd-timeStart))
            return True
        else:
            save_text_in_csv_log("Temperature Okay", str(timeEnd-timeStart))
            return False
    except:
        return False

#imageprocessing
def image_process(picture_file1, picture_file2):
    try:
        save_text_in_csv_log("Start of Checking Movement Between Pictures", "N/A")
        timeStart = float("%.20f" % time.time())
        # read images using imageio library into arrays and convert to graystyle, since we know all photos are not greystyle
        greystyle_image1 = average((imageio.imread(picture_file1).astype(float)), -1)
        greystyle_image2 = average((imageio.imread(picture_file2).astype(float)), -1)
        #https://stackoverflow.com/questions/49546179/python-normalize-image-exposure
        normalized_image1 = normalize_image(greystyle_image1)
        normalized_image2 = normalize_image(greystyle_image2)
        # Manhattan distance: http://dataaspirant.com/2015/04/11/five-most-popular-similarity-measures-implementation-in-python/
        difference = normalized_image1 - normalized_image2  # elementwise for scipy
        manhattan_norm = sum(abs(difference))
        timeEnd = float("%.20f" % time.time())
        #print(manhattan_norm/normalized_image1.size)
        if (int(manhattan_norm/normalized_image1.size) > 10):
            print("Movement between pictures detected")
            save_text_in_csv_log("Movement Between Pictures Detected", str(timeEnd-timeStart))
            return True
        else:
            save_text_in_csv_log("No Movement Detected", str(timeEnd-timeStart))
            return False
    except:
        print("Text file not analyzed")
        return False
    
    #print ("Zero norm:", n_0, "/ per pixel:", n_0*1.0/normalized_image2.size)

def normalize_image(picture_array):
    range_of_image = picture_array.max()-picture_array.min()
    min_value_of_array = picture_array.min()
    return (picture_array-min_value_of_array)*255/range_of_image
    

#Turn on Global Buzzer
def turn_on_buzzer():
    buzzer.on()

#Turn off Global Buzzer
def turn_off_buzzer():
    buzzer.off()

#Replace Latest File
def replace_latest_file(picture_file, target_file):
    shutil.copy(picture_file,target_file)

#Checks PIR and Door Relay
def check_sensors_for_motion():
    timeStart = float("%.20f" % time.time())
    event_occur = False
    if is_door_open():
        event_occur = True
    elif is_motion_detected():
        event_occur = True
    elif check_pressure_and_temperature_sensor():
        event_occur = True
    elif enable_picture_check_as_sensor:
        if not (file_name_last_analyzed == file_name_last_taken_picture):
            global file_name_last_analyzed
            file_name_last_analyzed = file_name_last_taken_picture
            if image_process(file_name_second_last_taken_picture, file_name_last_taken_picture):
                event_occur = True

    if event_occur:
        turn_on_buzzer()
        if enable_emails:
            if (not email_locking_semaphore):
                global email_locking_semaphore
                email_locking_semaphore = True
                if (len(file_name_last_taken_picture) > 1):
                    hold_the_camera = Thread(target=send_email_to(file_name_last_taken_picture)).start()
                
        #Clear queue so the files don't get deleted
        global saved_images_queue
        with saved_images_queue.mutex:
            saved_images_queue.queue.clear()

        if not save_next_five_picture_semaphore:
            save_the_next_pictures_thread = Thread(target=save_the_next_pictures).start()
    else:
        turn_off_buzzer()
    timeEnd = float("%.20f" % time.time())
    save_text_in_csv_log("All Sensor Check Time", str(timeEnd-timeStart))


def save_the_next_pictures():
    global save_next_five_picture_semaphore
    save_next_five_picture_semaphore = True
    while True:
        if (saved_images_queue.qsize() > 5):
            with saved_images_queue.mutex:
                saved_images_queue.queue.clear()
            break
    global save_next_five_picture_semaphore
    save_next_five_picture_semaphore = False

def append_value_to_keypad_string(keypress):
    global new_string
    new_string += str(keypress)
    #print(new_string)
    if keypress_incorrect_counter >= 4:
        print("Too many incorrect tries, wait for reset timer")
        global new_string
        new_string = ""
    else:
        global keypress_in_last_thirty_seconds
        keypress_in_last_thirty_seconds = True
        global keypress_thirty_second_cooldown
        keypress_thirty_second_cooldown = 30
        
        if("5675" in new_string or any(new_string in s for s in codes)):
            #print(codes)
            global new_string
            new_string = ""
            if (not keep_running_app_semaphore):
                global keep_running_app_semaphore
                keep_running_app_semaphore = True
                thread = threading.Thread(target=main)
                thread.start()
                print("Engaged the Alarm System")
                time.sleep(10)
                arm_alarm_lights()
                
            else:
                global keep_running_app_semaphore
                keep_running_app_semaphore = False
                print("Alarm System Deactivated")
                disarm_alarm_lights()
                turn_off_buzzer()
                
        if(len(new_string) >= 4):
            print("Incorrect")
            global new_string
            new_string = ""
            global keypress_incorrect_counter
            keypress_incorrect_counter += 1
            turn_on_buzzer()
            time.sleep(.5)
            turn_off_buzzer()

        if keypress_incorrect_counter >= 4:
            threadsleep = threading.Thread(target=hold_thirty_seconds_before_allowing_keypress_entry)
            threadsleep.start()
            #saveTextInFile("Incorrect Code Entered 4 times, system on lockdown")

def hold_thirty_seconds_before_allowing_keypress_entry():
    time.sleep(300)
    global keypress_incorrect_counter
    keypress_incorrect_counter = 0
    save_text_in_csv_log("Keypad", "300 seconds lockout ended")

def run_keypad():
    value = keypad.registerKeyPressHandler(append_value_to_keypad_string)

def thirty_second_reset_key():
    while True:
        time.sleep(1)
        if (keypress_in_last_thirty_seconds):
            global keypress_thirty_second_cooldown
            keypress_thirty_second_cooldown += -1
            #print(keypress_thirty_second_cooldown)

        if (keypress_thirty_second_cooldown == 0 and keypress_in_last_thirty_seconds):
            global keypress_in_last_thirty_seconds
            keypress_in_last_thirty_seconds = False
            global new_string
            new_string = ""
            save_text_in_csv_log("Keypad", "30 seconds elapsed without keypress, cleared code")
            print("time out over, can press again")
            

def start_keypad_threads():
    threadkeypad = threading.Thread(target=run_keypad)
    threadkeypad.start()
    threadkeypadCounter = threading.Thread(target=thirty_second_reset_key)
    threadkeypadCounter.start()

#based off: http://zetcode.com/python/smtplib/
#changed a few things to read the sendtoemail from a file so you can change it.
#added a thread to prevent email to be sent, the most would be 1 per 300 seconds, this can be changed with the code if necessary.
def send_email_to(attachmentfile):

    save_text_in_csv_log("Start Email Time", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
    timeStart = float("%.20f" % time.time())

    accountEmail = throw error
    accountPassword = throw error

    # Create a multipart message and set headers
    message = MIMEMultipart()
    message["From"] = accountEmail
    message["To"] = sendToEmail
    message["Subject"] = "WARNING: MOTION DETECTED FROM ALARM!"
    message.attach(MIMEText("SEE ATTACHED", "plain"))

    with open(attachmentfile, "rb") as attachment:
        part = MIMEBase("application", "octet-stream")
        part.set_payload(attachment.read())

    encoders.encode_base64(part)

    part.add_header('Content-Disposition', 'attachment', filename=attachmentfile)

    message.attach(part)
    text = message.as_string()

    context = ssl.create_default_context()
    with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as server:
        server.login(accountEmail, accountPassword)
        server.sendmail(accountEmail, sendToEmail, text)
        
    timeEnd = float("%.20f" % time.time())
    save_text_in_csv_log("Send Email Time", str(timeEnd-timeStart))
    hold_camera_thread = Thread(target=hold_thirty_seconds_before_allowing_another_email).start()
    print("did this print before hold")

def hold_thirty_seconds_before_allowing_another_email():
    time.sleep(10)
    global email_locking_semaphore
    email_locking_semaphore = False
    print("hold")

def run_camera():
    global is_camera_running_semaphore
    is_camera_running_semaphore = True
    while True:
        timeStart = float("%.20f" % time.time())
        global camera_in_use_semaphore
        camera_in_use_semaphore = True
        new_picture_file_name = current_working_directory + '/pictures/' + time.strftime("%Y%m%d-%H%M%S") + '.png'
  
        camera = PiCamera()
        camera.capture(new_picture_file_name)
        camera.close()

        global camera_in_use_semaphore
        camera_in_use_semaphore = False

        global saved_images_queue
        saved_images_queue.put(new_picture_file_name)

        global file_name_second_last_taken_picture
        file_name_second_last_taken_picture = file_name_last_taken_picture
        
        global file_name_last_taken_picture
        file_name_last_taken_picture = new_picture_file_name
        
        target_file = current_working_directory + '/static/latest.png'
        shutil.copy(file_name_last_taken_picture,target_file)
        
        if (saved_images_queue.qsize() > 10):
            global saved_images_queue
            ToDeleteFile = saved_images_queue.get()
            try:
                os.remove(ToDeleteFile)
            except:
                print("BORING")
            
        timeEnd = float("%.20f" % time.time())
        save_text_in_csv_log("Camera Save Picture", str(timeEnd-timeStart))

        if not keep_running_app_semaphore:
            turn_off_buzzer
            save_text_in_csv_log("Camera Shutdown", "")
            break
    global is_camera_running_semaphore
    is_camera_running_semaphore = False

          
#Always runs until semaphore of keep_running_app_semaphore is triggered
def main():
    if not is_camera_running_semaphore:
        camera_thread = Thread(target=run_camera).start()
    arm_alarm_lights()
    while True:
        check_sensors_for_motion()
        if not keep_running_app_semaphore:
            turn_off_buzzer
            disarm_alarm_lights()
            break
    turn_off_buzzer

########WEBSITE AND FLASK
#Basic flask commands from: http://flask.pocoo.org/
#Code is copied with running our commands in the middle from: http://flask.pocoo.org/docs/1.0/quickstart/


#shutdown app, used while testing, needed to kill flask due to errors, this code allows us to kill the localhost server
#gotten from: http://flask.pocoo.org/snippets/67/
def shutdown_server():
    shutdown_command = request.environ.get('werkzeug.server.shutdown')
    if shutdown_command is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    shutdown_command()

#this triggers the threads to start
@app.before_first_request
def activate_job():
    thread = threading.Thread(target=main)
    thread.start()
    start_keypad_threads()
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
    print("XXXXXXXXXXXXXXXXXXSTART THE SERVERXXXXXXXXXXXXXXXXXXXXXX")
    save_text_in_csv_log("Server Started", "XXXXXXXXXXXXXXXXXXXXXXX")

#Returns the main page
@app.route("/")
def main_html_page():
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }
    return render_template('main.html', **templateData)

#Loads page to stop the alarm system
@app.route("/kill")
def alarm_deactivate_page():
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }
    global keep_running_app_semaphore
    keep_running_app_semaphore = False
    turn_off_buzzer
    
    return render_template('main.html', **templateData)

@app.route("/start")
def alarm_activate_page():
    timeStart = float("%.20f" % time.time())
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }
    #checks if there is a current app running, if so, it will not start another one
    if (not keep_running_app_semaphore):
        global keep_running_app_semaphore
        keep_running_app_semaphore = True
        global email_locking_semaphore
        email_locking_semaphore = False
        thread = threading.Thread(target=main)
        thread.start()

    timeEnd = float("%.20f" % time.time())
    save_text_in_csv_log("Start Alarm From Website", str(timeEnd-timeStart))
    return render_template('main.html', **templateData)


@app.route("/takepicture")
def get_recent_picture_html():
    timeStart = float("%.20f" % time.time())
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }

    #takes the last picture and puts it in the latest.png
    if (len(file_name_last_taken_picture) > 1):
        target_file = current_working_directory + '/static/latest.png'
        shutil.copy(file_name_last_taken_picture,target_file)
    timeEnd = float("%.20f" % time.time())
    save_text_in_csv_log("Get Latest Picture from Website", str(timeEnd-timeStart))
    return render_template('main.html', **templateData)

#Switches on the send email function, slows down camera by 2-4 seconds on 1 instance while sensors are not read for about 7 seconds
@app.route("/emailon")
def set_email_on():
    global enable_emails
    enable_emails = True
    
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }

    return render_template('main.html', **templateData)

@app.route("/emailoff")
def set_email_off():
    global enable_emails
    enable_emails = False
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }

    return render_template('main.html', **templateData)

#Switches on the send email function, slows down camera by 2-4 seconds on 1 instance while sensors are not read for about 7 seconds
@app.route("/camerasensoron")
def set_camera_sensor_on():
    global enable_picture_check_as_sensor
    enable_picture_check_as_sensor = True
    
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }

    return render_template('main.html', **templateData)

@app.route("/camerasensoroff")
def set_camera_sensor_off():
    global enable_picture_check_as_sensor
    enable_picture_check_as_sensor = False
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M")
    templateData = {
        'title' : 'Sensor is off!',
        'time': timeString,
        'emailenabled' : enable_emails,
        'camerasensor' : enable_picture_check_as_sensor
    }

    return render_template('main.html', **templateData)

#html code
#<img src="{{url_for('static', csv_file_name='latest.png')}}" />
#shutdown for us to kill when testing because we keep failing
#gotten from: http://flask.pocoo.org/snippets/67/
@app.route("/shutdown")
def shutdown():
    shutdown_server()
    return 'Server shutting down...'

#Runner to start the flask load and load the methods to run
#Code from: https://networklore.com/start-task-with-flask/
def start_runner():
    def start_loop():
        not_started = True
        while not_started:
            try:
                start_runner_request = requests.get('http://127.0.0.1:5000/')
                if start_runner_request.status_code == 200:
                    print('Server started, quiting start_loop')
                    not_started = False
            except:
                print('Server not yet started')
            time.sleep(2)

    print('Started runner')
    thread = threading.Thread(target=start_loop)
    thread.start()


#cache
#https://stackoverflow.com/questions/34066804/disabling-caching-in-flask
@app.after_request
def add_header(request_cache):
    """
    Add headers to both force latest IE rendering engine or Chrome Frame,
    and also to cache the rendered page for 10 minutes.
    """
    request_cache.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    request_cache.headers["Pragma"] = "no-cache"
    request_cache.headers["Expires"] = "0"
    request_cache.headers['Cache-Control'] = 'public, max-age=0'
    return request_cache

#Here is how to run, added keyboard intterup
if __name__ == '__main__':
    keep_running_app_semaphore = True
    is_camera_running_semaphore = False
    file_name_last_taken_picture = ""
    try:
        start_runner()
        app.run(host='0.0.0.0', port=5000, debug = False, use_reloader = False)        
    except KeyboardInterrupt:
        turn_off_buzzer
        keep_running_app_semaphore = False
        raise
