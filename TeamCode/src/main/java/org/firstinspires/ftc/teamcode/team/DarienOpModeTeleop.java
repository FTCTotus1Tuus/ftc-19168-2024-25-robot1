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
//import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class DarienOpModeTeleop extends DarienOpMode {
@Override
    public void runOpMode() throws InterruptedException {}





    public void runDriveSystem() {
        direction[0] = -gamepad1.left_stick_x;
        direction[1] = -gamepad1.left_stick_y;
        rotation = -gamepad1.right_stick_x;
        turboBoost = gamepad1.left_stick_button;

        MoveRobot(direction, -rotation, turboBoost);
    }



    public void MoveRobot(double[] direction, double rotation, boolean turboBoost) {

        double divBy;
        double wheel0 = clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] + rotation, -1, 1);
        if (turboBoost) {
            divBy = turboDivBy;
        } else {
            divBy = regularDivBy;

        }
        telemetry.addData("", wheel0 / divBy);

        MoveMotor(omniMotor0, wheel0 / divBy);
        MoveMotor(omniMotor1, wheel1 / divBy);
        MoveMotor(omniMotor2, wheel2 / divBy);
        MoveMotor(omniMotor3, wheel3 / divBy);
    }
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
        omniMotor1 = initializeMotor("omniMotor3");
        omniMotor2 = initializeMotor("omniMotor1");
        omniMotor3 = initializeMotor("omniMotor2");

        //spinnerServo;

        //arm = initializeMotor("arm");
        // arm.setDirection(DcMotor.Direction.REVERSE);


        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);
    }
    int move_to_position;
    double y;

    public void print (String Name, Object message)
    {
        telemetry.addData(Name, message);
        telemetry.update();
    }
    public double relativePower ( double intended_power)
    {
        return (13 * intended_power) / getVoltage();
    }


    public DcMotor initializeMotor (String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void MoveMotor (DcMotor motor,double power){
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }
    public static double clamp ( double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
}