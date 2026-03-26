import datetime
import time
import math
import GPS_delta.py
import Remote.py

def PID_Distance(error_Distance, Integral_Distance,previous_Distance):
    Proportional_Distance = error_Distance
    Integral_Distance = (Integral_Distance + error_Distance) * dt
    Derivative_Distance = (error_Distance - previous_Distance) / dt
    previous_Distance = error_Distance
    output = (kp * Proportional_Distance) + (ki * Integral_Distance) + (kd * Derivative_Distance)
    return output

def PID_Angle(error_Angle, Integral_Angle,previous_Angle ):
    Proportional_Angle = error_Angle
    Integral_Angle = (Integral_Angle + error_Angle)*dt
    Derivative_Angle=(error_Angle-previous_Angle)/dt

    previous_Angle = error_Angle
    output= (kp*Proportional_Angle)+(ki*Integral_Angle)+(kd*Derivative_Angle)
    return output
    
verrou1 = threading.Lock()
verrou2 = threading.Lock()
kp = 0.8  # value to calculate
kd = 0.20  # value to calculate
ki = 0.001  # value to calculate

dt = 0
lastTime = 0

previous_Distance = 0
Proportional_Distance = 0
Integral_Distance = 0
Derivative_Distance = 0

previous_Angle = 0
Proportional_Angle = 0
Integral_Angle = 0
Derivative_Angle = 0


def GPS_serial(ser):
    while True:

        dataout = pynmea2.NMEAStreamReader()
        newdata = ser.readline()
        if (newdata[0:6] == b"$GPRMC"):
            try:
                newmsg = pynmea2.parse(newdata.decode("utf-8"))
            except:
                print(newmsg)
            finally:
                lat = newmsg.latitude
                lng = newmsg.longitude
            gps = str(lat) + ", " + str(lng)
        return ({"lat": lat, "lng": lng})


if __name__ == '__main__':

    ser = serial.Serial(port="/dev/ttyACM0", baudrate=57600, timeout=0.1)
    while True:
        with verrou1:
            gps_pos = GPS_serial(ser)
            print("found RMC " + str(gps))

        # ------------------------------------SETTINGS------------------------------------------------------
        M1_S1 = chip.get_line(MOTOR1_SENS1)
        M1_S1.request(consumer="M1_S1", type=gpiod.LINE_REQ_DIR_OUT)

        M1_S2 = chip.get_line(MOTOR1_SENS2)
        M1_S2.request(consumer="M1_S2", type=gpiod.LINE_REQ_DIR_OUT)

        M2_S1 = chip.get_line(MOTOR2_SENS1)
        M2_S1.request(consumer="M2_S1", type=gpiod.LINE_REQ_DIR_OUT)

        M2_S2 = chip.get_line(MOTOR2_SENS2)
        M2_S2.request(consumer="M2_S2", type=gpiod.LINE_REQ_DIR_OUT)

        pwm_tread = threading.Thread(target=pwm, args=(dutyCycleLeft, dutyCycleRight,))
        pwm_tread.start()

        A = {"Lat": lat, "Lon": lng}  # include here the stream from the GPS
        B = {"Lat": 47.39132, "Lon": -0.73924}  # include here the stream from the path wanted

        Robot_lenth = 0.70  # change le lenght by the robot true lenght
        Robot_width = 0.59  # change le width by the robot true width
        # --------------------------------------------------------------------------------------------------
        Antena_WeelAxis = Robot_lenth
        AntenaAxis_Weels = Robot_width / 2

        print(math.sqrt(Antena_WeelAxis) + math.sqrt(AntenaAxis_Weels))
        d = math.sqrt(Antena_WeelAxis ** 2 + AntenaAxis_Weels ** 2)

        # Δφ = ln( tan( latB / 2 + π / 4 ) / tan( latA / 2 + π / 4) )
        latA_rad = math.radians(A["Lat"])
        latB_rad = math.radians(B["Lat"])
        lonA_rad = math.radians(A["Lon"])
        lonB_rad = math.radians(B["Lon"])

        # Tau

        # start with A pos
        # A
        with verrou2:
            # 4.2 Converting the Joysticks signals to the Motor values (-255 to 255)
            dutyCycleRight = 255
            dutyCycleLeft = 255
            time.sleep(1)
            dutyCycleRight = 0
            dutyCycleLeft = 0
        A_plus_1 = {"Lat": 47.3915100, "Lon": -0.7390501}  # new A at T+1 value

        latA_plus_1_rad = math.radians(A_plus_1["Lat"])
        lonA_plus_1_rad = math.radians(A_plus_1["Lon"])

        Tau = DeltaGPS.angle_to_target_radius(latA_rad, lonA_rad, latA_plus_1_rad, lonA_plus_1_rad)  # Tau calculation

        Pos_Weel_Right = DeltaGPS.Pos_weel(latA_rad, lonA_rad, d, Tau)
        # if ROBOT goes to the full north: -Tau is correct but if it goes to the ouest, it is false -> TBD
        Pos_Weel_Left = DeltaGPS.Pos_weel(latA_rad, lonA_rad, d, Tau)

        print("Pos_Weel_Right :", Pos_Weel_Right)
        print("Pos_Weel_Left :", Pos_Weel_Left)

        # End of settings the TAU angle
        print("end!")
        time.sleep(10)
