package org.firstinspires.ftc.teamcode.team.testing;

//import static org.firstinspires.ftc.teamcode.team.TeamPropMaskPipeline.clawPositionL;
//import static org.firstinspires.ftc.teamcode.team.TeamPropMaskPipeline.clawPositionR;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.*;

import java.util.*;
import java.io.*;

@Disabled
@TeleOp

public class IntakeTest extends LinearOpMode{

    CRServo leftIntake;
    CRServo rightIntake;
    CRServo feeder;

    Servo clawWrist;
    Servo clawLeft;
    Servo clawRight;
    // l open is 0.4 closed is 0.1
    // r open is 0.6 closed is 0.9 for claw servos
    double clawLeftPositionOpen=0.4;
    double clawLeftPositionClosed=0.08;
    double clawRightPositionOpen=0.6;
    double clawRightPositionClosed=0.93;

    double[] direction = {0.0,0.0};
    double rotation;
    double regularDivBy = 1;
    double turboDivBy = 1;
    boolean turboBoost;

    DcMotor omniMotor0; // front left
    DcMotor omniMotor1; // front right
    DcMotor omniMotor2; // back left
    DcMotor omniMotor3; // back right

    DcMotor leftArm;
    DcMotor rightArm;

    public void runOpMode(){
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor3");
        omniMotor2 = initializeMotor("omniMotor1");
        omniMotor3 = initializeMotor("omniMotor2");

        leftArm = initializeMotor("leftArm");
        rightArm = initializeMotor("rightArm");

        leftArm.setDirection(DcMotor.Direction.REVERSE);

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
        waitForStart();

        //Start
        while (this.opModeIsActive()) {

            // DONE: TODO: Provide control to reverse the intake wheels so they can remove an unwanted pixel that gets sucked in.
            // DONE: TODO: gamepad1.right_bumper = intake; gamepad1.right_trigger = belt up.
            // DONE: TODO: gamepad1.left_bumper = reverse intake; gamepad1.left_trigger = belt down.
            if (gamepad1.right_bumper) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
            if (gamepad1.left_bumper) {
                feeder.setPower(-1);
            } else {
                feeder.setPower(0);
            }

            if(gamepad1.right_trigger>0) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
            if (gamepad1.left_trigger>0){
                feeder.setPower(1);
            } else {
                feeder.setPower(0);
            }
//          feeder.setPower((gamepad1.right_trigger ? 1:0) + (gamepad1.left_trigger ? -1:0));

            // TODO: Wrist to be on fixed positions for grabbing and for dropping on the backboard.
            if(gamepad2.right_trigger>0) {
                clawWrist.setPosition(0.7);
            } else {
                // do nothing
            }
            if (gamepad2.left_trigger>0){
                clawWrist.setPosition(0.3);
            } else {
                // do nothing
            }

            //clawWrist.setPower(gamepad2.left_stick_y/3); // clawWrist control as a continuous servo (CRServo).

//          clawLeft.setDirection(Servo.Direction.REVERSE);

//          if (gamepad2.a) {clawLeft.setPosition(0.8);} else {clawLeft.setPosition(1);}
//          clawLeft.setPosition(clawPositionL);
//          clawRight.setPosition(clawPositionR);
//          clawLeft.setPosition(gamepad2.right_stick_x);
//          clawRight.setPosition(gamepad2.right_stick_y);

        if(gamepad2.right_bumper) {
            // Open the claw
            clawLeft.setPosition(clawLeftPositionOpen);
            clawRight.setPosition(clawRightPositionOpen);
        } else if (gamepad2.left_bumper){
            // Close the claw
            clawLeft.setPosition(clawLeftPositionClosed);
            clawRight.setPosition(clawRightPositionClosed);
        }

            // DONE: TODO: Arm control on the left joystick up/down.
            leftArm.setPower(-gamepad2.left_stick_y);
            rightArm.setPower(-gamepad2.left_stick_y);
            //leftArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            //rightArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            direction[0] = -gamepad1.left_stick_x;
            direction[1] = -gamepad1.left_stick_y;
            rotation = -gamepad1.right_stick_x;
            turboBoost = gamepad1.left_stick_button;

            MoveRobot(direction, -rotation, turboBoost);
        }
        /*
        if macro = true and running = false
        running = true

        if running = true
            tiny power close claw hands
            switch deltatime

            100ms
            retract claw

            200ms
            raise arm

            300ms

         */

    }

    public void MoveRobot(double[] direction, double rotation, boolean turboBoost){

        double divBy;
        double wheel0 = clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = clamp(direction[0] + -direction[1] + rotation, -1, 1);
        if (turboBoost) {
            divBy = turboDivBy;
        }
        else {
            divBy = regularDivBy;

        }
        telemetry.addData("", wheel0/divBy);

        MoveMotor(omniMotor0,wheel0/divBy);
        MoveMotor(omniMotor1,wheel1/divBy);
        MoveMotor(omniMotor2,wheel2/divBy);
        MoveMotor(omniMotor3,wheel3/divBy);
    }

    public DcMotor initializeMotor(String name){
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void MoveMotor(DcMotor motor, double power){
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}