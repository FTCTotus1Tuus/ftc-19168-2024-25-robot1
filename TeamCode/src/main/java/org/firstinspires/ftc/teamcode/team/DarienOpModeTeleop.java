package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class DarienOpModeTeleop extends DarienOpMode {


    public double[] direction = {0.0, 0.0};
    public double rotation;
    /**
     *  If the GP1 left bumper is pressed, spin the boot wheels in if the joystick is pulled down or spin the boot wheels out if the joystick is pushed up.
     */
    public void runIntakeSystem() {

        if (gamepad1.right_bumper) {
            print("INTAKE", "Load sample");
            intakeWheels.setPower(1);
        } else if (gamepad1.x) {
            print("INTAKE", "Eject sample");
            intakeWheels.setPower(-0.5);
        } else {
            // Stop
            intakeWheels.setPower(0);
        }


        if (gamepad1.left_bumper){
            intakeSlide.setPower(-gamepad1.right_stick_y);
        }

        if (gamepad1.right_bumper) {
            intakeWrist.setPosition(intakeWristGroundPosition);
        }
        else {
            intakeWrist.setPosition(iintakeWristUpPosition);
        }
    }

    public void runVerticalSlideSystem() {

        verticalSlide.setPower(-gamepad2.left_stick_y);

        if (gamepad2.a) {
            bucket.setPosition(bucketPlace);
        } else {
            bucket.setPosition(bucketPickup);
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

    public void MoveMotor (DcMotor motor,double power){
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }

    public static double clamp ( double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }

}