package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class DarienOpModeTeleop extends DarienOpMode {


    public double[] direction = {0.0, 0.0};
    public double rotation;
    public static double verticalSlideMaxHeight = 4400;
    public static double lift1MaxHeight = 3700;
    public double durationSecondsIntakeSlideIn = 2;
    public boolean startedIntakeSlide = false;
    public double startTime = 0;

//    public void pollSensors() {
//        //intakeColorSensor.enableLed(true);
//        telemetry.addData("R: ", intakeColorSensor.red());
//        telemetry.addData("G: ", intakeColorSensor.green());
//        telemetry.addData("B: ", intakeColorSensor.blue());
////        telemetry.addData("Distance:", intakeColorSensor) // Possibly use the proximity sensor values from the color sensor.
//        telemetry.update();
//
//        int[] COLOR_RED = {0, 0, 0};
//        int[] COLOR_BLUE = {0, 0, 0};
//        int[] COLOR_YELLOW = {0, 0, 0};
//
//    }

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
            // First, center the sampleYaw servo.
            sampleYawSetPosition(POS_SAMPLE_YAW_CENTER);
            //sleep(200); // let the servo go to position.
            samplePitch.setPosition(POS_SAMPLE_PITCH_PICKUP_READY);
            intakeWristSetPosition(intakeWristUpPosition);

            // TODO: Instead of using a timer, use the intakeWristTouchSensor to stop running the intakeSlide servo.
            // Pull the intakeSlide in for only x seconds to avoid burning out the intakeSlide servo.
            intakeSlide.setPower(powerIntakeSlideIn);
            startTime = getRuntime();
            startedIntakeSlide = true;
        } else {
            // INDEPENDENT CONTROLS

            // CONTROL: SAMPLE CLAW
            if (gamepad2.left_trigger > 0.5) {
                sampleClaw.setPosition(sampleClawOpen);
            } else {
                sampleClaw.setPosition(sampleClawClosed);
            }

            telemetry.addData("intakeWrist pos: ", intakeWristGetPosition());
            telemetry.addData("sampleYaw pos: ", sampleYawGetPosition());
            telemetry.update();

            if (gamepad2.dpad_left) {
                sampleYawSetPosition(sampleYawGetPosition() - 0.005);
            } else if (gamepad2.dpad_right) {
                sampleYawSetPosition(sampleYawGetPosition() + 0.005);
            }

            if (intakeWristGetPosition() >= intakeWristGroundPosition - 0.1) {
                if (gamepad2.y) {
                    // lower the samplePitch for picking up samples from floor
                    samplePitch.setPosition(POS_SAMPLE_PITCH_PICKUP);
                } else {
                    // lift the samplePitch for going in/out of the sub (slightly above the floor samples)
                    samplePitch.setPosition(POS_SAMPLE_PITCH_PICKUP_READY);
                }
            } else if (intakeWristGetPosition() <= intakeWristUpPosition + 0.1) {
                samplePitch.setPosition(POS_SAMPLE_PITCH_DROP_BUCKET);
            } else {
                samplePitch.setPosition(POS_SAMPLE_PITCH_PICKUP_READY);
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
                intakeWristSetPosition(intakeWristUpPosition);
                sampleYawSetPosition(POS_SAMPLE_YAW_CENTER);
            } else if (gamepad1.right_trigger > 0.05) {
                intakeWristSetPosition(intakeWristGroundPosition);
                sampleYawSetPosition(POS_SAMPLE_YAW_CENTER);
            }
//            } else if (gamepad1.right_trigger > 0.05 && intakeWrist.getPosition() > intakeWristUpPosition) {
//                // Allow driver to lift the intake wrist slightly, but don't go beyond the max position.
//                double targetIntakeWristPosition = intakeWristGroundPosition - (gamepad1.right_trigger / 5);
//                if (targetIntakeWristPosition > intakeWristUpPosition) {
//                    intakeWrist.setPosition(targetIntakeWristPosition);
//                }
//            }
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
        double lift_power, lift_iduty, lift_pduty, lift_pgain = .002, lift_igain = .00001, lift_sp = 50;
        boolean doingL2ascent = false;
        // encoder values are positive and motor is in forward mode
        if (gamepad2.dpad_up && lift1.getCurrentPosition() <= lift1MaxHeight) {
            lift1.setPower(1);
            specimenWrist.setPosition(specimenWristPlace);
        } else if (gamepad2.dpad_down /* && lift1.getCurrentPosition() >= 0*/) {
            lift_power = 1;
            lift_iduty = -lift_power;
            lift1.setPower(-lift_power);
            if (gamepad2.x) {
                double intakeSlideStartTime = getRuntime();
                doingL2ascent = true;
                while (doingL2ascent && !gamepad2.b) {
                    lift_pduty = clamp(lift_pgain * (lift_sp - lift1.getCurrentPosition()), -.5, .5);
                    lift_iduty = clamp(lift_igain * (lift_sp - lift1.getCurrentPosition()) + lift_iduty, -.7, .7);
                    lift_power = clamp(lift_pduty + lift_iduty, -.8, .8);
                    lift1.setPower(lift_power);
                    telemetry.addData("Pduty: ", lift_pduty);
                    telemetry.addData("Iduty: ", lift_iduty);
                    telemetry.addData("lift1: ", lift1.getCurrentPosition());
                    telemetry.update();
                    if (gamepad2.b) {
                        doingL2ascent = false;
                    }
                    if (getRuntime() - intakeSlideStartTime <= 30) {
                        /*while within 30 seconds set intake slide power 0.1 */
                        intakeSlide.setPower(-0.1); //to keep intake slide from rolling out while doing the L2 ascent
                        intakeWristSetPosition(intakeWristUpPosition);
                    } else {
                        /* otherwise set intake slide power to 0*/
                        intakeSlide.setPower(0);
                        intakeWristSetPosition(intakeWristUpPosition);
                    }

                }
            }
        } else {
            lift1.setPower(0);
        }
        telemetry.addData("Lift 1 Pos: ", lift1.getCurrentPosition());
        //telemetry.update();
        //allow driver to reset encoder to 0 for the lift 1 slide
        if (gamepad2.back && gamepad2.x) {
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void liftLock() {

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
//        if (turboBoost) {
//            divBy = turboDivBy;
//        } else {
//            divBy = regularDivBy;
//
//        }

        divBy = (gamepad1.left_trigger / 2) + 0.5;
        telemetry.addData("", wheel0 * divBy);

        MoveMotor(omniMotor0, wheel0 * divBy);
        MoveMotor(omniMotor1, wheel1 * divBy);
        MoveMotor(omniMotor2, wheel2 * divBy);
        MoveMotor(omniMotor3, wheel3 * divBy);
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