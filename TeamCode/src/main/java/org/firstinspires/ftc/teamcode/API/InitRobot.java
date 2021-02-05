package org.firstinspires.ftc.teamcode.API;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.Encoder;
import org.firstinspires.ftc.teamcode.API.HW.SmartMotor;
import org.firstinspires.ftc.teamcode.API.HW.SmartServo;

import java.util.HashMap;
import java.util.Objects;

/**
 * The Robot Initializer. Place initialization code here. This prevents needing to sync the init code
 * between all OpModes.
 */
public class InitRobot {
    public static final boolean MODE_4x4 = true; // True if you are using 4x4 drive
    private static SmartMotor bl, br, fl, fr, flywheel, intake;
    private static Encoder leftEncoder, rightEncoder, lateralEncoder;

    // TODO: JavaDoc
    public static void init(LinearOpMode l) {
        /*
        * #######                   ######
        * #       #####  # #####    #     # ###### #       ####  #    #
        * #       #    # #   #      #     # #      #      #    # #    #
        * #####   #    # #   #      ######  #####  #      #    # #    #
        * #       #    # #   #      #     # #      #      #    # # ## #
        * #       #    # #   #      #     # #      #      #    # ##  ##
        * ####### #####  #   #      ######  ###### ######  ####  #    #
        */

        // Get motors
        bl = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BL_NAME));
        br = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BR_NAME));
        if (MODE_4x4) {
            fl = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FL_NAME));
            fr = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FR_NAME));
        }
        flywheel = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FLYWHEEL));
        //intake = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_LEFT));
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        HashMap<String, SmartMotor> motors = new HashMap<>();
        motors.put(Naming.MOTOR_BL_NAME, bl);
        motors.put(Naming.MOTOR_BR_NAME, br);
        if (MODE_4x4) {
            motors.put(Naming.MOTOR_FL_NAME, fl);
            motors.put(Naming.MOTOR_FR_NAME, fr);
        }
        motors.put(Naming.MOTOR_FLYWHEEL, flywheel);
        //motors.put(Naming.MOTOR_INTAKE, intake);

        // Get servos
        SmartServo wobbleArm = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_WOBBLE_ARM_NAME));
        SmartServo wobbleGrabber = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_WOBBLE_GRABBER_NAME));
        SmartServo launcher = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_LAUNCHER));
        //SmartServo grabber = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_GRABBER_NAME));
        //SmartServo rotate = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_ROTATE_NAME));
        //SmartServo fLeftNew = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_FOUNDATION_LEFT_NEW_NAME));
        //SmartServo fRightNew = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_FOUNDATION_RIGHT_NEW_NAME));

        //fLeftNew.setDirection(Servo.Direction.REVERSE);
        //fRightNew.setDirection(Servo.Direction.FORWARD);

        // Add servos into the list
        HashMap<String, SmartServo> servos = new HashMap<>();
        servos.put(Naming.SERVO_WOBBLE_ARM_NAME, wobbleArm);
        servos.put(Naming.SERVO_WOBBLE_GRABBER_NAME, wobbleGrabber);
        servos.put(Naming.SERVO_LAUNCHER, launcher);
        //servos.put(Naming.SERVO_GRABBER_NAME, grabber);
        //servos.put(Naming.SERVO_ROTATE_NAME, rotate);
        //servos.put(Naming.SERVO_FOUNDATION_LEFT_NEW_NAME, fLeftNew);
        //servos.put(Naming.SERVO_FOUNDATION_RIGHT_NEW_NAME, fRightNew);

        // Get CRServos
        //CRServo armExtend = l.hardwareMap.get(CRServo.class, Naming.CRSERVO_EXTEND_NAME);

        // Add CRServos into the list
        HashMap<String, CRServo> crServos = new HashMap<>();
        //crServos.put(Naming.CRSERVO_EXTEND_NAME, armExtend);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        if (MODE_4x4) {
            fl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.REVERSE);
        }

        // Set motors to spin in the correct direction
        //sc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (MODE_4x4) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (MODE_4x4) {
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Switch direction of servo
        //rotate.setDirection(Servo.Direction.REVERSE);

        // Get color sensors
        //ColorSensor scissorDownLimit = l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_DOWN_LIMIT_NAME);
        //ColorSensor scissorUpLimit = l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_UP_LIMIT_NAME);
        NormalizedColorSensor parkSensor = (NormalizedColorSensor)l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_PARK);

        // Add color sensors into list
        HashMap<String, NormalizedColorSensor> colorSensors = new HashMap<>();
        //colorSensors.put(Naming.COLOR_SENSOR_DOWN_LIMIT_NAME, scissorDownLimit);
        //colorSensors.put(Naming.COLOR_SENSOR_UP_LIMIT_NAME, scissorUpLimit);
        colorSensors.put(Naming.COLOR_SENSOR_PARK, parkSensor);

        // Get webcams
        //WebcamName webcam1 = l.hardwareMap.get(WebcamName.class, Naming.WEBCAM_0_NAME);

        // Add webcams to list
        HashMap<String, WebcamName> webcams = new HashMap<>();
        //webcams.put(Naming.WEBCAM_0_NAME, webcam1);

        // Get gyros
        BNO055IMU gyro0 = l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_0_NAME);
        BNO055IMU gyro1 = l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_1_NAME);

        // Add gyros to list
        HashMap<String, BNO055IMU> gyros = new HashMap<>();
        gyros.put(Naming.GYRO_0_NAME, gyro0);
        gyros.put(Naming.GYRO_1_NAME, gyro1);

        // Get dead wheel encoders
        leftEncoder = new Encoder(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_LEFT));
        rightEncoder = new Encoder(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_RIGHT));
        lateralEncoder = new Encoder(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_LATERAL));

        // Add encoders to list
        HashMap<String, Encoder> encoders = new HashMap<>();
        encoders.put(Naming.ENCODER_LEFT, leftEncoder);
        encoders.put(Naming.ENCODER_RIGHT, rightEncoder);
        encoders.put(Naming.ENCODER_LATERAL, lateralEncoder);

        // Add lists into the movement class
        Movement movement = new Movement();
        Movement.motors = motors;
        Movement.servos = servos;
        Movement.crServos = crServos;

        Sensor sensor = new Sensor();
        Sensor.gyros = gyros;
        Sensor.colorSensors = colorSensors;
        Sensor.webcams = webcams;
        Sensor.encoders = encoders;

        Robot.movement = movement;
        Robot.sensor = sensor;
        Robot.linearOpMode = l;

        // Send power to servos so they don't move
        for (String key : servos.keySet()) {
            Robot.movement.setServo(key, Objects.requireNonNull(servos.get(key)).getPosition());
        }

        // Initialize gyros
        for (String key : gyros.keySet()) {
            Robot.sensor.initGyro(key);
        }
    }
}

