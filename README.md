# DC MOTOR TUNING
```bash 
This project is developed to perform dc motor tuning for speed control, you can tune both motors at a time , which is very helpful to tune motors for differential drive robot
```
### HARDWARE USED : 
```bash
- Arduino Nano 33 IoT
- L298N motor driver
- 11.1v Lipo battery
- 12v 130 RPM DC motors
```
### HOW TO PERFORM MOTOR TUNING :

#### 1. First connect your motors to arduino , you can see the connections in arduino sketch
```bash 
    Left_ENC_A -> 2
    Left_ENC_B -> 3
    ENA        -> 11
    IN1        -> 7
    IN2        -> 8

    RIGHT_ENC_A -> 9
    RIGHT_ENC_B -> 10
    ENB         -> 6
    IN3         -> 4
    IN4         -> 5
```
#### 2. Once connections are made, upload the sketch into arduino, then connect to your laptop
#### 3. Connect the battery to motors
#### 4. Run pid_tuner.py script and connect to select the arduino port in tuner GUI and connect to it
#### 5. Set Target RPM and tune PID values 
#### 6. Everytime the values you set are saved to pid_settings.json file, whenever you close and reopen the app , it will automatically take the previously set values

### Note : 
#### - modify the counts per revolution as per your motors