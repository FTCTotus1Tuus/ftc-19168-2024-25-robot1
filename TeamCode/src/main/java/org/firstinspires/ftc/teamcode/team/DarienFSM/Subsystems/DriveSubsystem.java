package org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.team.DarienHelperFunctions;
import org.firstinspires.ftc.teamcode.team.GoBildaPinpointDriver;

public class DriveSubsystem extends SubsystemBase {
    //TODO add a telemetry function to send all data we need to dash and driver hub


    private MotorEx omniMotor0, omniMotor1, omniMotor2, omniMotor3;

    private Telemetry telemetry;

    private GoBildaPinpointDriver pinpoint;

    private HardwareMap hm;

    private double[] direction;
    private double rotation;
    private boolean turboBoost;

    //CONSTS
    public static double rotConst = 1;
    public static double acceptableXYError = 0.25; //how many inches off the xy movement can be - does not compound
    public static double minimumXYspeed = 5;
    public static double movement_igain = 0;
    public static double movement_pgain = 0.06;
    public static double distanceToSlowdown = 4; //Inches
    public static double slowdownPower = 0.35;


    //AUTO Variables
    private double targetX, targetY, targetRot;
    private Pose2D currentRobotPos;

    public DriveSubsystem(MotorEx omniMotor0, MotorEx omniMotor1, MotorEx omniMotor2, MotorEx omniMotor3, Telemetry telemetry, GoBildaPinpointDriver pinpoint, HardwareMap hm) {
        this.omniMotor0 = omniMotor0;
        this.omniMotor1 = omniMotor1;
        this.omniMotor2 = omniMotor2;
        this.omniMotor3 = omniMotor3;
        this.telemetry = telemetry;
        this.pinpoint = pinpoint;
        this.hm = hm;
    }

    public void teleopDrive(double forward, double strafe, double rotation, double turbo) {
        strafe = Math.pow(-strafe, 5);
        forward = Math.pow(-forward, 5); //TODO possibly flip neg signs
        rotation = Math.pow(rotation, 5);

        MoveRobot(strafe, forward, rotation, turbo);

    }

    public void setAutoDestination(double x, double y, double rot) {
        this.targetX = x;
        this.targetY = y;
        this.targetRot = rot;
    }


    public boolean autoUpdatePower(double maxPower, boolean noPID, boolean noSlowdown, double errorBand) {
        //Returns if robot is within the error margins and should go on to the next command

        double errorX, errorY, errorXp, errorYp, errorH, errorHrads;
        double movement_pduty, movement_iduty = 0, movement_power;
        Pose2D velocity;

        updatePosition(); // VERY NESSCESSARY WHENEVER WE ARE MOVING

        errorX = targetX - getXPos();
        errorY = targetY - getYPos();
        errorH = getErrorRot(targetRot);
        errorHrads = Math.toRadians(errorH) * 7;

        if (Math.abs(errorH) <= 5) {// attempts to make sure jitters happen less
            errorH = 0; // if error is within 5 degrees on either side we say we're good
        }

        errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
        errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));


        if (noPID) {
            if (DarienHelperFunctions.getHypotenuse(errorXp, errorYp) < distanceToSlowdown && !noSlowdown) {
                setPower(slowdownPower, errorXp, errorYp, errorHrads); // add pid?
                telemetry.addData("slow speed - no pid", "");
            } else {
                setPower(maxPower, errorXp, errorYp, errorHrads); // add pid?
                telemetry.addData("full speed - no pid", "");
            }
        } else {
            if (DarienHelperFunctions.getHypotenuse(errorXp, errorYp) < distanceToSlowdown && !noSlowdown) {
                setPower(slowdownPower, errorXp, errorYp, errorHrads); // add pid?
                telemetry.addData("final approach - pid", "");
            } else {

                movement_pduty = DarienHelperFunctions.clamp(movement_pgain * Math.pow(DarienHelperFunctions.getHypotenuse(errorXp, errorYp), 3 / 2), -1, 1);
                movement_iduty = DarienHelperFunctions.clamp(movement_igain * (DarienHelperFunctions.getHypotenuse(errorXp, errorYp)) + movement_iduty, -.7, .7);
                movement_power = DarienHelperFunctions.clamp(movement_pduty + movement_iduty, -maxPower, maxPower);
                setPower(movement_power, errorXp, errorYp, errorHrads);
                telemetry.addData("current move power: ", movement_power);
            }
        }
        //exit controls
        if (DarienHelperFunctions.getHypotenuse(errorX, errorY) <= errorBand) {
            return true;
        } else if (DarienHelperFunctions.getHypotenuse(pinpoint.getVelX(), pinpoint.getVelY()) <= minimumXYspeed &&
                DarienHelperFunctions.getHypotenuse(errorX, errorY) < acceptableXYError * 4) {
            return true;
        }
//        else if ((this.time - movementStartTime) > timeout) {
//            return true;
//        } REMOVING FOR NOW ADD BACK LATER IF NESSCESSARY

        return false;
    }

    private void setPower(double power, double adjX, double adjY, double adjH) {

        double[] motorPowers = scalePower(
                (adjY + adjX - adjH * rotConst),
                (adjY - adjX + adjH * rotConst),
                (adjY - adjX - adjH * rotConst),
                (adjY + adjX + adjH * rotConst), power);

        omniMotor0.setVelocity(DarienHelperFunctions.relativePower(motorPowers[0], hm));
        omniMotor1.setVelocity(DarienHelperFunctions.relativePower(motorPowers[1], hm));
        omniMotor2.setVelocity(DarienHelperFunctions.relativePower(motorPowers[2], hm));
        omniMotor3.setVelocity(DarienHelperFunctions.relativePower(motorPowers[3], hm));
    }

    private double[] scalePower(double motorPower0, double motorPower1, double motorPower2, double motorPower3, double power) {
        double maxPower = Math.max(Math.max(Math.abs(motorPower0), Math.abs(motorPower1)), Math.max(Math.abs(motorPower2), Math.abs(motorPower3)));
        if (maxPower > power) {
            motorPower0 = (motorPower0 * power) / maxPower;
            motorPower1 = (motorPower1 * power) / maxPower;
            motorPower2 = (motorPower2 * power) / maxPower;
            motorPower3 = (motorPower3 * power) / maxPower;
        }

        double[] returnPower = new double[]{
                motorPower0, motorPower1, motorPower2, motorPower3
        };
        return returnPower;
    }

    private void MoveRobot(double strafe, double forward, double rotation, double speedBoost) {

        double divBy;
        double wheel0 = DarienHelperFunctions.clamp(-strafe + forward + rotation, -1, 1);
        double wheel1 = DarienHelperFunctions.clamp(strafe + forward - rotation, -1, 1);
        double wheel2 = DarienHelperFunctions.clamp(-strafe + -forward - rotation, -1, 1);
        double wheel3 = DarienHelperFunctions.clamp(strafe + -forward + rotation, -1, 1);

        divBy = (speedBoost / 2) + 0.5;
        telemetry.addData("", wheel0 * divBy);

        MoveMotor(omniMotor0, wheel0 * divBy);
        MoveMotor(omniMotor1, wheel1 * divBy);
        MoveMotor(omniMotor2, wheel2 * divBy);
        MoveMotor(omniMotor3, wheel3 * divBy);
    }


    private void MoveMotor(MotorEx motor, double power) {
        motor.setVelocity(power);
    }

    private void updatePosition() {
        pinpoint.update();
        currentRobotPos = pinpoint.getPosition();
    }

    private double getRawHeading() {
        return currentRobotPos.getHeading(AngleUnit.DEGREES);
    }

    private double getErrorRot(double targetPosRot) {
        // pos is clockwwise neg is counterclockwise
        return ((targetPosRot - getRawHeading()) + 180) % 360 - 180;
    }

    private double getXPos() {
        return currentRobotPos.getX(DistanceUnit.INCH);
    }

    private double getYPos() {
        return currentRobotPos.getY(DistanceUnit.INCH);
    }

}
