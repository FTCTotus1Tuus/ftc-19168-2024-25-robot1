package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class DarienOpModeTeleop extends DarienOpMode {


    public double[] direction = {0.0, 0.0};
    public double rotation;
    public static double verticalSlideMaxHeight = 4400;
    public static double lift1MaxHeight = 4000;
    public double durationSecondsIntakeSlideIn = 2;
    public boolean startedIntakeSlide = false;
    public double startTime = 0;

    public void pollSensors() {
        intakeColorSensor.enableLed(true);
        telemetry.addData("R: ", intakeColorSensor.red());
        telemetry.addData("G: ", intakeColorSensor.green());
        telemetry.addData("B: ", intakeColorSensor.blue());
//        telemetry.addData("Distance:", intakeColorSensor) // Possibly use the proximity sensor values from the color sensor.
        telemetry.update();

        int[] COLOR_RED = {0, 0, 0};
        int[] COLOR_BLUE = {0, 0, 0};
        int[] COLOR_YELLOW = {0, 0, 0};

    }

    /**
     * If the GP1 left bumper is pressed, spin the boot wheels in if the joystick is pulled down or spin the boot wheels out if the joystick is pushed up.
     */
    public void runIntakeSystem() {
        if (startedIntakeSlide) {
            if (getRuntime() - startTime > durationSecondsIntakeSlideIn) {
                intakeSlide.setPower(0);
                startedIntakeSlide = false;
            }
        }
        if (gamepad1.a) {
            // MACRO: Raise the sample that has been scooped.
            intakeWheels.setPower(0);
            intakeWrist.setPosition(intakeWristUpPosition);

            // Pull the intakeSlide in for only x seconds to avoid burning out the intakeSlide servo.
            intakeSlide.setPower(powerIntakeSlideIn);
            startTime = getRuntime();
            startedIntakeSlide = true;
        } else {
            // INDEPENDENT CONTROLS

            // CONTROL: INTAKE WHEELS
            if (gamepad1.right_bumper) {
                print("INTAKE", "Load sample");
                intakeWheels.setPower(powerIntakeWheelToPickupSample);
            } else if (gamepad1.x) {
                print("INTAKE", "Eject sample");
                intakeWheels.setPower(powerIntakeWheelToEjectSample);
            } else if (gamepad1.left_trigger != 0) {
                intakeWheels.setPower(-gamepad1.left_trigger / 2);
            } else if (gamepad2.left_trigger > 0.1) {
                intakeWheels.setPower(-gamepad2.left_trigger);
                print("INTAKE", "Eject sample");
            } else {
                // Stop
                intakeWheels.setPower(0);
            }

            // CONTROL: INTAKE SLIDE
            if (!startedIntakeSlide) {
                if (gamepad1.left_bumper) {
                    // Control the intake slide with the right stick if and only if the left bumper is pressed.
                    intakeSlide.setPower(-gamepad1.right_stick_y);
                } else {
                    intakeSlide.setPower(0);
                }
            }

            // CONTROL: INTAKE WRIST
            if (gamepad1.right_bumper) {
                intakeWrist.setPosition(intakeWristGroundPosition);
            } else if (gamepad1.right_trigger > 0.05 && intakeWrist.getPosition() > intakeWristUpPosition) {
                // Allow driver to lift the intake wrist slightly, but don't go beyond the max position.
                double targetIntakeWristPosition = intakeWristGroundPosition - (gamepad1.right_trigger / 5);
                if (targetIntakeWristPosition > intakeWristUpPosition) {
                    intakeWrist.setPosition(targetIntakeWristPosition);
                }
            }
        }
    }

    public void runVerticalSlideSystem() {


        if (verticalSlide.getCurrentPosition() > verticalSlideMaxHeight && -gamepad2.left_stick_y > 0) {
            verticalSlide.setPower(0.1);
        } else if (Math.abs(gamepad2.left_stick_y) < 0.02) {
            verticalSlide.setPower(0);
        } else {
            verticalSlide.setPower(-gamepad2.left_stick_y);
        }
        telemetry.addData("vslide posL ", verticalSlide.getCurrentPosition());
        telemetry.addData("zero power behavior", verticalSlide.getZeroPowerBehavior());
        print("vslide power", verticalSlide.getPower());

        // adds a driver override to the height cap code
        if (gamepad2.back && gamepad2.b) {
            verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.a) {
            bucket.setPosition(bucketPlace);
        } else {
            bucket.setPosition(bucketPickup);
        }
    }

    public void runLift1System() {
        // encoder values are positive and motor is in forward mode
        if (gamepad2.dpad_up && lift1.getCurrentPosition() <= lift1MaxHeight) {
            //if (gamepad2.dpad_up) {
            lift1.setPower(1);
        } else if (gamepad2.dpad_down && lift1.getCurrentPosition() >= 0) {
            //} else if (gamepad2.dpad_down) {
            lift1.setPower(-1);
        } else {
            lift1.setPower(0);
        }
        telemetry.addData("Lift 1 Pos: ", lift1.getCurrentPosition());
        //telemetry.update();
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
        direction[0] = Math.pow(-gamepad1.left_stick_x, 5);
        direction[1] = Math.pow(-gamepad1.left_stick_y, 5);
        if (!gamepad1.left_bumper) {
            rotation = Math.pow(-gamepad1.right_stick_x, 5);
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