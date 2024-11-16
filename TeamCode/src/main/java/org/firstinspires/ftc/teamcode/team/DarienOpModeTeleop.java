package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class DarienOpModeTeleop extends DarienOpMode {


    public double[] direction = {0.0, 0.0};
    public double rotation;

    /**
     * If the GP1 left bumper is pressed, spin the boot wheels in if the joystick is pulled down or spin the boot wheels out if the joystick is pushed up.
     */
    public void runIntakeSystem() {

        // CONTROL: INTAKE WHEELS
        if (gamepad1.right_bumper) {
            print("INTAKE", "Load sample");
            intakeWheels.setPower(powerIntakeWheelToPickupSample);
       /* } else if (gamepad1.x) {
            print("INTAKE", "Eject sample");
            intakeWheels.setPower(powerIntakeWheelToEjectSample);*/
        } else if (!gamepad1.x) {
            // Stop
            intakeWheels.setPower(0);
        } intakeWheels.setPower(-gamepad1.left_trigger / 2);

        // CONTROL: INTAKE SLIDE
        if (gamepad1.left_bumper) {
            // Control the intake slide with the right stick if and only if the left bumper is pressed.
            intakeSlide.setPower(-gamepad1.right_stick_y);
        } else if (gamepad1.a) {
            // Pull the intake slide IN only for a few seconds to avoid running the servo continuously and burning it out.
            intakeSlide.setPower(powerIntakeSlideIn);

            /* TODO: This code causes the robot to freeze when button A is pressed. Needs fixing.
            double durationSecondsIntakeSlideIn = 4;
            double startTime = getRuntime();
            while ((getRuntime() - startTime) < durationSecondsIntakeSlideIn) {
                intakeSlide.setPower(powerIntakeSlideIn);
            }
             */
        }

        if (gamepad1.right_bumper) {
            intakeWrist.setPosition(intakeWristGroundPosition);
        } else if (gamepad1.a) {
            intakeWheels.setPower(0);
            intakeWrist.setPosition(intakeWristUpPosition);
        }


        intakeWrist.setPosition(intakeWrist.getPosition() - gamepad1.right_trigger / 5);
    }

    public void runVerticalSlideSystem() {
        verticalSlide.setPower(-gamepad2.left_stick_y);
        if (Math.abs(gamepad2.left_stick_y) < 0.02) {
            verticalSlide.setPower(0);
        }
        telemetry.addData("zero power behavior", verticalSlide.getZeroPowerBehavior());
        print("vslide power", verticalSlide.getPower());

        if (gamepad2.a) {
            bucket.setPosition(bucketPlace);
        } else {
            bucket.setPosition(bucketPickup);
        }
    }

    public void runSpecimenSystem() {

        if (-gamepad2.right_stick_y > 0.5) {
            specimenWrist.setPosition(specimenWristPlace);
        } else if (-gamepad2.right_stick_y < -0.5) {
            specimenWrist.setPosition(specimenWristPickup);
        }

        if (gamepad2.right_trigger > 0.5) {
            specimenClaw.setPosition(specimenClawOpen);
        } else {
            specimenClaw.setPosition(specimenClawClosed);
        }

    }

    public void runDriveSystem() {
        direction[0] = -gamepad1.left_stick_x;
        direction[1] = -gamepad1.left_stick_y;
        if (!gamepad1.left_bumper) {
            rotation = -gamepad1.right_stick_x;
        }
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

    public void MoveMotor(DcMotor motor, double power) {
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

}