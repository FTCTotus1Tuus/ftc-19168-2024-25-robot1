package org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.DarienHelperFunctions;
import org.firstinspires.ftc.teamcode.team.GoBildaPinpointDriver;

public class DriveSubsystem extends SubsystemBase {

    private DcMotor omniMotor0, omniMotor1, omniMotor2, omniMotor3;

    private Telemetry telemetry;

    private GoBildaPinpointDriver pinpoint;

    private double[] direction;
    private double rotation;
    private boolean turboBoost;

    public DriveSubsystem(DcMotor omniMotor0, DcMotor omniMotor1, DcMotor omniMotor2, DcMotor omniMotor3, Telemetry telemetry, GoBildaPinpointDriver pinpoint) {
        this.omniMotor0 = omniMotor0;
        this.omniMotor1 = omniMotor1;
        this.omniMotor2 = omniMotor2;
        this.omniMotor3 = omniMotor3;
        this.telemetry = telemetry;
        this.pinpoint = pinpoint;
    }

    public void teleopDrive(Gamepad drivePad) {
        direction[0] = Math.pow(-drivePad.left_stick_x, 5);
        direction[1] = Math.pow(-drivePad.left_stick_y, 5);
        if (!drivePad.left_bumper) {
            rotation = Math.pow(-drivePad.right_stick_x, 5);
        }
        turboBoost = drivePad.left_stick_button;
        MoveRobot(direction, -rotation, turboBoost, drivePad);

    }

    private void MoveRobot(double[] direction, double rotation, boolean turboBoost, Gamepad drivePad) {

        double divBy;
        double wheel0 = DarienHelperFunctions.clamp(-direction[0] + direction[1] + rotation, -1, 1);
        double wheel1 = DarienHelperFunctions.clamp(direction[0] + direction[1] - rotation, -1, 1);
        double wheel2 = DarienHelperFunctions.clamp(-direction[0] + -direction[1] - rotation, -1, 1);
        double wheel3 = DarienHelperFunctions.clamp(direction[0] + -direction[1] + rotation, -1, 1);

        divBy = (drivePad.left_trigger / 2) + 0.5;
        telemetry.addData("", wheel0 * divBy);

        MoveMotor(omniMotor0, wheel0 * divBy);
        MoveMotor(omniMotor1, wheel1 * divBy);
        MoveMotor(omniMotor2, wheel2 * divBy);
        MoveMotor(omniMotor3, wheel3 * divBy);
    }


    private void MoveMotor(DcMotor motor, double power) {
        motor.setPower(power);
    }


}
