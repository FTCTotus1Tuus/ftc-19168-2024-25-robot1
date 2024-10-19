package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class DarienOpMode extends LinearOpMode {

    // HARDWARE COMPONENTS
    public DcMotor omniMotor0; // left front
    public DcMotor omniMotor1; // right front
    public DcMotor omniMotor2; // left rear
    public DcMotor omniMotor3; // right rear
    public DcMotor verticalSlide;
    public IMU imu;
    public CRServo intakeSlide;
    public Servo intakeWrist;
    public CRServo intakeWheels;
    //public ColorSensor intakeColorSensor;
    //public TouchSensor intakeWristTouchSensor;

    // HARDWARE FIXED CONSTANTS
    public static double encoderResolution = 537.7 ; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches
    public double constMult = (wheelDiameter * (Math.PI));
    public double constant = encoderResolution / constMult;
    public static double rotationTolerance = 5;
    public static double power = 0.3;

    // HARDWARE TUNING CONSTANTS
    public int encoderPos0, encoderPos1, encoderPos2, encoderPos3;
    public double[] direction = {0.0, 0.0};
    public double rotation;
    public int encoderPos;
    public double regularDivBy = 1;
    public double turboDivBy = 1;
    public boolean turboBoost;


    @Override
    public void runOpMode() throws InterruptedException {}

    public void initControls(boolean isAuto) {
        //isAuto: true=auto false=teleop
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();

        // INITIALIZE MOTORS
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");

        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);

        //verticalSlide = initializeMotor("verticalSlide");
        //verticalSlide.setDirection(DcMotor.Direction.FORWARD);

        // INITIALIZE SERVOS
        intakeSlide = hardwareMap.get(CRServo.class, "intakeSlide"); // CH port 3
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist"); // CH port 1
        intakeWheels = hardwareMap.get(CRServo.class, "intakeWheels"); // CH port 2

        // INITIALIZE DIFFERENTLY FOR AUTONOMOUS
        if (isAuto) {
           // arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //colourSensorLeft = hardwareMap.get(ColorSensor.class, "colourSensorLeft");
            //colourSensorRight = hardwareMap.get(ColorSensor.class, "colourSensorRight");
        }
    }

    public double getVoltage () {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public void print (String Name, Object message)
    {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        telemetry.update();
    }

    public double relativePower ( double intended_power)
    {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage();
    }

    public DcMotor initializeMotor (String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

}