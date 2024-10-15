package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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

    public static double encoderResolution = 537.7 ; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches
    public double constMult = (wheelDiameter * (Math.PI));
    public double constant = encoderResolution / constMult;
    public static double rotationTolerance = 5;
    public static double power = 0.3;
    public int encoderPos0, encoderPos1, encoderPos2, encoderPos3;
    public double[] direction = {0.0, 0.0};
    public double rotation;
    public int encoderPos;
    public double regularDivBy = 1;
    public double turboDivBy = 1;
    public boolean turboBoost;
    public DcMotor omniMotor0; // leftFront
    public DcMotor omniMotor1; // leftRear
    public DcMotor omniMotor2; // rightRear
    public DcMotor omniMotor3; // rightFront
    //public DcMotor arm;
    public IMU imu;


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

        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");



        if (isAuto) {
           // arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //colourSensorLeft = hardwareMap.get(ColorSensor.class, "colourSensorLeft");
            //colourSensorRight = hardwareMap.get(ColorSensor.class, "colourSensorRight");
        }


        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);





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
         it just initializes the motor*/
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return motor;
        }

    }