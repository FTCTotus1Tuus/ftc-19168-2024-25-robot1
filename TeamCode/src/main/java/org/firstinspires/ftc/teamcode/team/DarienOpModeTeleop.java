package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class DarienOpModeTeleop extends DarienOpMode {
        @Override
    public void runOpMode() throws InterruptedException {}

    public void runDroneSystem(){
        if(gamepad2.left_stick_button && gamepad2.b){
            droneLauncher.setPower(0.8);
            sleep(500);
        } else {
            droneLauncher.setPower(0);
        }
    }
    public void runIntakeSystem() {
        if (gamepad1.right_bumper) {
            // Load pixels
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        } else if (gamepad1.right_trigger > 0) {
            // Eject pixels
            leftIntake.setPower(-0.5);
            rightIntake.setPower(0.5);
        } else {
            // Stop
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void runFeederSystem() {
        if (gamepad1.left_bumper) {
            // Load pixels
            feeder.setPower(-1);
        } else if (gamepad1.left_trigger > 0) {
            // Eject pixels
            feeder.setPower(1);
        } else {
            // Stop
            feeder.setPower(0);
        }
    }

    public void runWristSystem() {
        // Wrist to be on fixed positions for grabbing and for dropping on the backboard.
        if (gamepad2.right_trigger > 0) {
            // Pick up pixels from the robot tray.
            setWristPosition("pickup");
        }
        if (gamepad2.left_trigger > 0) {
            // Drop pixels on backboard
            setWristPosition("drop");
        }
        if (gamepad2.x) {
            setWristPosition("dropGround");
        }
        //clawWrist.setPower(gamepad2.left_stick_y/3); // clawWrist control as a continuous servo (CRServo).
    }

    public void setWristPosition(String position) {
        switch (position) {
            case "pickup":
                clawWrist.setPosition(clawWristPositionPickup);
                break;
            case "drop":
                clawWrist.setPosition(clawWristPositionDrop);
                break;
            case "dropGround":
                clawWrist.setPosition(clawWristPositionGround);
                break;
            default:
                // do nothing;
        }
    }

    public void runClawSystem() {
        if (gamepad2.right_bumper) {
            setClawPosition("open");
            // Open the claw
        } else if (gamepad2.left_bumper) {
            setClawPosition("closed");
            // Close the claw
        }

    }

    public void setClawPosition(String position) {
        switch (position) {
            case "open":
                clawLeft.setPosition(clawLeftPositionOpen);
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "closed":
                clawLeft.setPosition(clawLeftPositionClosed);
                clawRight.setPosition(clawRightPositionClosed);
                break;
            case "leftOpen":
                clawLeft.setPosition(clawLeftPositionOpen);
                break;
            case "rightOpen":
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "leftClosed":
                clawLeft.setPosition(clawLeftPositionClosed);
                break;
            case "rightClosed":
                clawRight.setPosition(clawRightPositionClosed);
                break;
            default:
                // do nothing;
        }
    }

    public void runArmSystem() {
        // DONE: Arm control on the left joystick up/down.
        leftArm.setPower(-gamepad2.left_stick_y);
        rightArm.setPower(-gamepad2.left_stick_y);
    }

    public void driveArm(String position) {
        double power = 0.8;
        switch (position) {
            case "in":
                // TODO: power the arm down until it reaches the stopping position. Then turn off the power.
                leftArm.setPower(-power);
                rightArm.setPower(-power);
                break;
            case "out":
                leftArm.setPower(power);
                rightArm.setPower(power);
                break;
            case "none":
                leftArm.setPower(0);
                rightArm.setPower(0);
            default:
                // do nothing;
        }
    }

    public void runDriveSystem() {
        direction[0] = (Math.pow(-gamepad1.left_stick_x, 7) -gamepad1.left_stick_x)/2;
        direction[1] = (Math.pow(-gamepad1.left_stick_y, 7) -gamepad1.left_stick_y)/2;
        rotation = (Math.pow(-gamepad1.right_stick_x, 7) -gamepad1.right_stick_x) / 2;
        turboBoost = gamepad1.left_stick_button;

        MoveRobot(direction, -rotation, turboBoost);
    }

    public void runMacro(String macro) {
        switch (macro) {
            case "ReadyToPickup":
                if (gamepad2.y) {
                    if (clawWrist.getPosition() < clawWristPositionPickup) {
                        // If the wrist is already in PICKUP position, do nothing.
                        setWristPosition("pickup");
                    }
                    if (clawLeft.getPosition() < clawLeftPositionOpen) {
                        // If the claw is already in OPEN position, do nothing.
                        setClawPosition("open");
                    }
                }
                while (gamepad2.y) {
                    driveArm("in");
                }
                break;
            case "ReadyToDrop":
                if (gamepad2.a) {
                    if (clawWrist.getPosition() > clawWristPositionDrop) {
                        // Only raise the wrist if it's not already in position.
                        setWristPosition("drop");
                    }
                }
                while (gamepad2.a) {
                    driveArm("out");
                }
                break;
            default:
                // do nothing;
                break;
        }
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

        leftArm = initializeMotor("leftArm");
        rightArm = initializeMotor("rightArm");

        leftArm.setDirection(DcMotor.Direction.REVERSE);

        if (isAuto) {
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            colourSensorLeft = hardwareMap.get(ColorSensor.class, "colourSensorLeft");
            colourSensorRight = hardwareMap.get(ColorSensor.class, "colourSensorRight");
        }


        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        feeder = hardwareMap.get(CRServo.class, "feeder");
        droneLauncher = hardwareMap.get(CRServo.class, "droneLauncher");
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