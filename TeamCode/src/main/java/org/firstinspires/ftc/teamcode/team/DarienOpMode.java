// To run on the robot:
// 1. Make Module   (Top ribbon)
// 2. Run           (Top ribbon)

package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DarienOpMode extends LinearOpMode {

    // HARDWARE COMPONENTS
    public DcMotor omniMotor0; // left front
    public DcMotor omniMotor1; // right front
    public DcMotor omniMotor2; // left rear
    public DcMotor omniMotor3; // right rear
    public DcMotor verticalSlide;
    public IMU imu;
    public GoBildaPinpointDriver odo;
    public CRServo intakeSlide;
    public Servo intakeWrist;
    //public CRServo intakeWheels;
    //public ColorSensor intakeColorSensor;
    public TouchSensor intakeWristTouchSensor;
    public Servo sampleClaw;
    public Servo sampleYaw;
    public Servo samplePitch;
    public Servo bucket;
    public Servo cameraWrist;
    public Servo specimenWrist;
    public Servo specimenClaw;
    public SparkFunOTOS myOtos;
    public DcMotor lift1;

    // HARDWARE FIXED CONSTANTS
    public static double encoderResolution = 537.7; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches
    public double constMult = (wheelDiameter * (Math.PI));
    public double inchesToEncoder = encoderResolution / constMult;
    public static double rotationTolerance = 2; //in degrees
    public static double power = 0.3;
    public static double powerIntakeWheelToPickupSample = 0.9;
    public static double powerIntakeWheelToEjectSample = -0.25;
    public static double powerIntakeSlideIn = -0.45;
    public static double PI = 3.1416;

    // HARDWARE TUNING CONSTANTS
    public int encoderPos0, encoderPos1, encoderPos2, encoderPos3;
    public int encoderPos;
    public double regularDivBy = 2;
    public double turboDivBy = 1;
    public boolean turboBoost;

    // Servo tuning constants

    public static double bucketPickup = 0.6;
    public static double bucketPlace = 0.45;
    public static double bucketUp = 0.9;

    public static double specimenWristPlace = 0.20; // towards inside of robot - change name later?
    public static double specimenWristPickup = 0.82;

    public static double POS_SAMPLE_PITCH_DROP_BUCKET = 0.2;
    public static double POS_SAMPLE_PITCH_ARM_DOWN = 0.4;
    public static double POS_SAMPLE_PITCH_PICKUP_READY = 0.43;
    public static double POS_SAMPLE_PITCH_PICKUP = 0.65;

    public static double POS_SAMPLE_YAW_LEFT_MAX = 0.3;
    public static double POS_SAMPLE_YAW_LEFT2 = 0.3;
    public static double POS_SAMPLE_YAW_LEFT1 = 0.4;
    public static double POS_SAMPLE_YAW_CENTER = 0.5;
    public static double POS_SAMPLE_YAW_RIGHT1 = 0.6;
    public static double POS_SAMPLE_YAW_RIGHT2 = 0.7;
    public static double POS_SAMPLE_YAW_RIGHT_MAX = 0.8;

    public static double sampleClawOpenWide = 0.95;
    public static double sampleClawOpen = 0.85;
    public static double sampleClawClosed = 0.74;

    public static double intakeWristGroundPosition = 0.7;
    public static double intakeWristUpPosition = 0.43;

    //vertical slide positions
    public static int highChamberBelowPos = 1750;
    public static int highChamberPlacePos = 2300;

    // calibrated for torque servo
    public static double specimenClawOpen = 0.94;
    public static double specimenClawClosed = 0.83;

    // Variables to track servo position
    private double sampleYawCurrentPosition;
    private double intakeWristCurrentPosition;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void initControls() {

        // INITIALIZE SENSORS
        //intakeColorSensor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
        intakeWristTouchSensor = hardwareMap.get(TouchSensor.class, "intakeWristTouchSensor");

        // Initialize IMU on the REV Control Hub
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
        imu.resetYaw();

//        // Initialize the SparkFun Odometry Tracking Optical Sensor (OTOS), which includes an IMU.
//        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
//        configureOtos();

        // Initialize 2 Deadwheel odometry
        configure2DeadWheel();

        // INITIALIZE MOTORS
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");

        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);

        verticalSlide = initializeMotor("verticalSlide");
        verticalSlide.setDirection(DcMotor.Direction.REVERSE);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1 = initializeMotor("lift1");
        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // INITIALIZE SERVOS
        intakeSlide = hardwareMap.get(CRServo.class, "intakeSlide"); // Expansion Hub port 0
        //cameraWrist = hardwareMap.get(Servo.class, "cameraWrist"); // CH port 0
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist"); // CH port 1
        //intakeWheels = hardwareMap.get(CRServo.class, "intakeWheels"); // CH port 2
        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw"); // CH port 4
        specimenWrist = hardwareMap.get(Servo.class, "specimenWrist"); // CH port 5
        bucket = hardwareMap.get(Servo.class, "bucket"); // CH port 3
        sampleClaw = hardwareMap.get(Servo.class, "sampleClaw"); // CH port
        samplePitch = hardwareMap.get(Servo.class, "samplePitch"); // CH port
        sampleYaw = hardwareMap.get(Servo.class, "sampleYaw"); // CH port

        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }

    public void sampleYawSetPosition(double position) {
        if (POS_SAMPLE_YAW_LEFT_MAX <= position && position <= POS_SAMPLE_YAW_RIGHT_MAX) {
            sampleYaw.setPosition(position);
            sampleYawCurrentPosition = position;
        }
    }

    public double sampleYawGetPosition() {
        return sampleYawCurrentPosition;
    }

    public void intakeWristSetPosition(double position) {
        intakeWrist.setPosition(position);
        intakeWristCurrentPosition = position;
    }

    public double intakeWristGetPosition() {
        return intakeWristCurrentPosition;
    }

    public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public void print(String Name, Object message) {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        //telemetry.update();
    }

    public double relativePower(double intended_power) {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage();
    }

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void configure2DeadWheel() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(0, 165); //these are tuned for 2/5/2025 robot

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO check and fix these values


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Y offset", odo.getXOffset());
        telemetry.addData("X offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -0.25, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(0.969);
        myOtos.setAngularScalar(1.000);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();
//        while (myOtos.getImuCalibratio

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);


        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));

        telemetry.update();
        print("ready to start", "");
    }


    public double getHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double getHypotenuse(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}