package org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.team.DarienHelperFunctions.initializeMotor;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
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

    //CONSTS
    private static double rotConst = 1;
    private static double acceptableXYError = 0.25; //how many inches off the xy movement can be - does not compound
    private static double minimumXYspeed = 5;
    private static double movement_igain = 0;
    private static double movement_pgain = 0.06;
    private static double distanceToSlowdown = 4; //Inches
    private static double slowdownPower = 0.35;
    private static double errorBand = 1;
    private double errorX, errorY, errorXp, errorYp, errorH, errorHrads;
    private boolean noPID, noSlowdown;
    private double maxPower;


    //AUTO Variables
    private double targetX, targetY, targetRot;
    private Pose2D currentRobotPos;

    public DriveSubsystem(String omniMotor0, String omniMotor1, String omniMotor2, String omniMotor3, Telemetry telemetry, String pinpoint, HardwareMap hm) {
        this.hm = hm;
        this.telemetry = telemetry;

        this.omniMotor0 = initializeMotor(omniMotor0, hm);
        this.omniMotor1 = initializeMotor(omniMotor1, hm);
        this.omniMotor2 = initializeMotor(omniMotor2, hm);
        this.omniMotor3 = initializeMotor(omniMotor3, hm);

        this.omniMotor0.setInverted(true); //is direction to reverse
        this.omniMotor1.setInverted(false);
        this.omniMotor2.setInverted(false);
        this.omniMotor3.setInverted(true);

        configurePinpoint(pinpoint);
    }

    public void teleDrive(double forward, double strafe, double rotation, double turbo) {
        strafe = Math.pow(strafe, 5);
        forward = Math.pow(-forward, 5);
        rotation = Math.pow(rotation, 5);

        MoveRobot(strafe, forward, rotation, turbo);

    }

    public void setAutoDestination(double x, double y, double rot, boolean noPID, boolean noSlowdown, double maxPower) {
        this.targetX = x;
        this.targetY = y;
        this.targetRot = rot;
        this.noPID = noPID;
        this.noSlowdown = noSlowdown;
        this.maxPower = maxPower;

    }


    public void setAutoDestination(double x, double y, double rot) {
        setAutoDestination(x, y, rot, false, false, 1);
    }

    public void autoUpdatePower() {

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
    }


    public boolean isMovementDone() {
        if (DarienHelperFunctions.getHypotenuse(errorX, errorY) <= errorBand) {
            return true;
        } else if (DarienHelperFunctions.getHypotenuse(pinpoint.getVelX(), pinpoint.getVelY()) <= minimumXYspeed &&
                DarienHelperFunctions.getHypotenuse(errorX, errorY) < acceptableXYError * 4) {
            return true;
        }

        return false;
    }

    private void setPower(double power, double adjX, double adjY, double adjH) {

        double[] motorPowers = scalePower(
                (adjY + adjX - adjH * rotConst),
                (adjY - adjX + adjH * rotConst),
                -(adjY - adjX - adjH * rotConst),
                -(adjY + adjX + adjH * rotConst), power);

        telemetry.addData("0 move power: ", motorPowers[0]);
        telemetry.addData("1 move power: ", motorPowers[1]);
        telemetry.addData("2 move power: ", motorPowers[2]);
        telemetry.addData("3 move power: ", motorPowers[3]);

        omniMotor0.setRunMode(Motor.RunMode.RawPower); //TODO look at switching to velocity instead
        omniMotor1.setRunMode(Motor.RunMode.RawPower);
        omniMotor2.setRunMode(Motor.RunMode.RawPower);
        omniMotor3.setRunMode(Motor.RunMode.RawPower);

        omniMotor0.set(motorPowers[0]);
        omniMotor1.set(motorPowers[1]);
        omniMotor2.set(motorPowers[2]);
        omniMotor3.set(motorPowers[3]);
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

        omniMotor0.setRunMode(Motor.RunMode.RawPower);
        omniMotor1.setRunMode(Motor.RunMode.RawPower);
        omniMotor2.setRunMode(Motor.RunMode.RawPower);
        omniMotor3.setRunMode(Motor.RunMode.RawPower);

        MoveMotor(omniMotor0, wheel0 * divBy);
        MoveMotor(omniMotor1, wheel1 * divBy);
        MoveMotor(omniMotor2, wheel2 * divBy);
        MoveMotor(omniMotor3, wheel3 * divBy);
    }


    private void MoveMotor(MotorEx motor, double power) {
        motor.set(power);
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

    private void configurePinpoint(String name) {
        pinpoint = hm.get(GoBildaPinpointDriver.class, name);

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        pinpoint.setOffsets(0, 165); //these are tuned for 2/5/2025 robot

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO check and fix these values


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Y offset", pinpoint.getXOffset());
        telemetry.addData("X offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
    }
}
