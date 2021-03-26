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
import org.firstinspires.ftc.teamcode.API.Config.Constants;
import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.Encoder;
import org.firstinspires.ftc.teamcode.API.HW.SmartColorSensor;
import org.firstinspires.ftc.teamcode.API.HW.SmartMotor;
import org.firstinspires.ftc.teamcode.API.HW.SmartServo;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Objects;

/**
 * The Robot Initializer. Place initialization code here. This prevents needing to sync the init code
 * between all OpModes.
 */
public class InitRobot {
    public static final boolean MODE_4x4 = true; // True if you are using 4x4 drive
    private static SmartMotor fl;
    private static SmartMotor fr;

    // TODO: JavaDoc
    public static void init(@NotNull LinearOpMode l) {
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
        SmartMotor bl = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BL));
        SmartMotor br = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BR));
        if (MODE_4x4) {
            fl = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FL));
            fr = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FR));
        }
        SmartMotor flywheel = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FLYWHEEL));
        SmartMotor intake = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_LEFT));
        SmartMotor intake2 = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_INTAKE2));
        SmartMotor wobbleArm = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_WOBBLE_ARM));
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HashMap<String, SmartMotor> motors = new HashMap<>();
        motors.put(Naming.MOTOR_BL, bl);
        motors.put(Naming.MOTOR_BR, br);
        if (MODE_4x4) {
            motors.put(Naming.MOTOR_FL, fl);
            motors.put(Naming.MOTOR_FR, fr);
        }
        motors.put(Naming.MOTOR_FLYWHEEL, flywheel);
        motors.put(Naming.MOTOR_INTAKE, intake);
        motors.put(Naming.MOTOR_INTAKE2, intake2);
        motors.put(Naming.MOTOR_WOBBLE_ARM, wobbleArm);

        // Get servos
        SmartServo wobbleGrabber = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_WOBBLE_GRABBER));
        SmartServo launcher = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_LAUNCHER));

        // Add servos into the list
        HashMap<String, SmartServo> servos = new HashMap<>();
        //servos.put(Naming.SERVO_WOBBLE_ARM, wobbleArm);
        servos.put(Naming.SERVO_WOBBLE_GRABBER, wobbleGrabber);
        servos.put(Naming.SERVO_LAUNCHER, launcher);

        // Add CRServos into the list
        HashMap<String, CRServo> crServos = new HashMap<>();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        if (MODE_4x4) {
            fl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.REVERSE);
        }
        
        intake2.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to spin in the correct direction
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

        bl.setPowerModifier(Constants.MOTOR_BL_POWERMOD);
        br.setPowerModifier(Constants.MOTOR_BR_POWERMOD);
        fl.setPowerModifier(Constants.MOTOR_FL_POWERMOD);
        fr.setPowerModifier(Constants.MOTOR_FR_POWERMOD);

        // Get color sensors
        SmartColorSensor parkSensor = new SmartColorSensor((NormalizedColorSensor)l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_PARK));
        parkSensor.setRedFudge(Constants.PARK_RED_FUDGE);
        parkSensor.setGreenFudge(Constants.PARK_GREEN_FUDGE);
        parkSensor.setBlueFudge(Constants.PARK_BLUE_FUDGE);

        SmartColorSensor armSensor = new SmartColorSensor((NormalizedColorSensor)l.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_ARM));
        armSensor.setRedFudge(Constants.ARM_RED_FUDGE);
        armSensor.setGreenFudge(Constants.ARM_GREEN_FUDGE);
        armSensor.setBlueFudge(Constants.ARM_BLUE_FUDGE);

        // Add color sensors into list
        HashMap<String, SmartColorSensor> colorSensors = new HashMap<>();
        colorSensors.put(Naming.COLOR_SENSOR_PARK, parkSensor);
        colorSensors.put(Naming.COLOR_SENSOR_ARM, armSensor);

        // Get webcams
        WebcamName webcam1 = l.hardwareMap.get(WebcamName.class, Naming.WEBCAM_0);

        // Add webcams to list
        HashMap<String, WebcamName> webcams = new HashMap<>();
        webcams.put(Naming.WEBCAM_0, webcam1);

        // Get gyros
        BNO055IMU gyro0 = l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_0);
        BNO055IMU gyro1 = l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_1);

        // Add gyros to list
        HashMap<String, BNO055IMU> gyros = new HashMap<>();
        gyros.put(Naming.GYRO_0, gyro0);
        gyros.put(Naming.GYRO_1, gyro1);

        // Get dead wheel encoders
        Encoder leftEncoder = new Encoder(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_LEFT));
        Encoder rightEncoder = new Encoder(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_RIGHT));
        Encoder lateralEncoder = new Encoder(l.hardwareMap.get(DcMotorEx.class, Naming.ENCODER_LATERAL));

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
        
        Robot.state = new StateMachine();

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