package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//import org.apache.commons.math3.geometry.euclidean.twod.Line;


@Config
public class DarienOpModeAuto extends DarienOpMode {

    public static double movement_igain = 0;
    public static double movement_pgain = 0.06;
    public static double distanceToSlowdown = 4; //Inches
    public static double slowdownPower = 0.35;

    public static double normalPower = 0.3;
    public static double verticalSlidePower = 1; //swapped to 1 from 0.8 needs testing
    public static double strafingInefficiencyFactor = 1.145;

//    public static double sampleClawClosed = 0.83;

    public double movementStartTime;

    public Pose2D currentRobotPos;

    //vertical slide positions
    public static int barBelow2Pos;
    public static int barPlace2Pos;
    public static int barBelow1Pos;


    FtcDashboard dashboard;


    //encoder movement targets
    public double targetPosX = 0;
    public double targetPosY = 0;
    public double targetPosH = 0;
    public double rotConst = 1;
    public double acceptableXYError = 0.25; //how many inches off the xy movement can be - does not compound
    public static double minimumXYspeed = 5;
    public double currentMovementPower = 0;
    public static double ProportionalCoefficient = 0.3;

    public double currentHeading = 0;

    public double specimenWristUp = 0.48;

    public double currentPosition = 0;
    public static double rotationEncoderConstant = 557;

    @Override
    public void initControls() {
        super.initControls();


        // Initialize the servo positions and record the current position.
        sampleYawSetPosition(POS_SAMPLE_YAW_CENTER);
        intakeWristSetPosition(intakeWristUpPosition);

//        print("otos", myOtos);

        // reverse motors 2 and 3
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);

        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this is key

        odo.resetPosAndIMU();

        dashboard = FtcDashboard.getInstance();


    }


    public void setVerticalSlide(String where, double power) {
        switch (where) {
            case "high chamber below":
                verticalSlide.setTargetPosition(highChamberBelowPos);
                break;
            case "high chamber place":
                verticalSlide.setTargetPosition(highChamberPlacePos);
                break;
            case "2nd bar below":
                verticalSlide.setTargetPosition(barBelow2Pos);
                break;
            case "2nd bar place":
                verticalSlide.setTargetPosition(barPlace2Pos);
                break;
            case "1st bar below":
                verticalSlide.setTargetPosition(barBelow1Pos);
                break;
            case "1st bar place":
                verticalSlide.setTargetPosition(barPlace1Pos);
                break;
            case "basket low":
                verticalSlide.setTargetPosition(basketLowPos);
                break;
            case "basket high":
                verticalSlide.setTargetPosition(basketHighPos);
                break;
            case "low":
                verticalSlide.setTargetPosition(armGroundPos);
                break;
            default:
                throw new RuntimeException("invalid position for vertical slide");
        }
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlide.setPower(power);
    }

    public void setSpecimenClaw(String position) {
        switch (position) {
            case "closed":
                specimenClaw.setPosition(specimenClawClosed);
                break;
            case "open":
                specimenClaw.setPosition(specimenClawOpen);
                break;
            default:
                throw new RuntimeException("invalid position for specimen claw");

        }
    }

    public void setSpecimenWrist(String position) {
        switch (position) {
            case "pickup":
                specimenWrist.setPosition(specimenWristPickup);
                break;
            case "place":
                specimenWrist.setPosition(specimenWristPlace);
                break;
            case "up":
                specimenWrist.setPosition(specimenWristUp);
                break;
            default:
                throw new RuntimeException("invalid position for specimen wrist");

        }
    }

    public void setIntakeWrist(String position) {
        switch (position) {
            case "down":
                intakeWrist.setPosition(intakeWristGroundPosition);
                break;
            case "up":
                intakeWrist.setPosition(intakeWristUpPosition);
                break;
            default:
                throw new RuntimeException("invalid position for intake wrist");
        }
    }

    public void setBucketPosition(String position) {
        switch (position) {
            case "carry":
                bucket.setPosition(bucketPickup);
                break;
            case "drop":
                bucket.setPosition(bucketPlace);
                break;
            case "up":
                bucket.setPosition(bucketUp);
                break;
            default:
                throw new RuntimeException("invalid position for bucket position");
        }
    }

    public void startIntake() {
        //intakeWheels.setPower(powerIntakeWheelToPickupSample);
    }

    public void setIntakeSlidePower(double power) {
        intakeSlide.setPower(power);
    }

    public void setSamplePitch(double position) {
        samplePitch.setPosition(position);
    }

    public void setSamplePitch(String position) {
        switch (position) {
            case "drop":
                samplePitch.setPosition(POS_SAMPLE_PITCH_DROP_BUCKET);
                break;
            case "arm down":
                samplePitch.setPosition(POS_SAMPLE_PITCH_ARM_DOWN);
                break;
            case "ready":
                samplePitch.setPosition(POS_SAMPLE_PITCH_PICKUP_READY);
                break;
            case "ground":
                samplePitch.setPosition(POS_SAMPLE_PITCH_PICKUP);
                break;
            default:
                throw new RuntimeException("invalid position for sample pitch");
        }
    }

    public void setSampleClaw(String position) {
        switch (position) {
            case "closed":
                sampleClaw.setPosition(sampleClawClosed);
                break;
            case "open":
                sampleClaw.setPosition(sampleClawOpen);
                break;
            case "openwide":
                sampleClaw.setPosition(sampleClawOpenWide);
                break;
            default:
                throw new RuntimeException("invalid position for sample claw");
        }
    }

    public void readyToPickupSample() {
        setIntakeWrist("down");
        setSamplePitch("ready");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        setSampleClaw("openwide");
    }

    public void pickupSample() {
        setSamplePitch("ground");
        sleep(300);
        setSampleClaw("closed");
    }

    public void placeSampleInBucket() {
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        setIntakeWrist("up");
        setSamplePitch("drop");
    }

    public void stopIntake() {
        //intakeWheels.setPower(0);
    }

    public void reverseIntake() {
        //intakeWheels.setPower(-0.5);
    }

    public void reverseIntake(double power) {
        //intakeWheels.setPower(power);
    }


    public void moveToPosition(double globalX, double globalY, double power) {
        moveToPosition(globalX, globalY, currentHeading, power);
    }

    public void moveToPosition(double globalX, double globalY, double globalH, double power) {
        // uses optical sensor to move by setting robot motor power
        // DOES NOT USE ENCODERS

        updatePosition(); //VERY NESSCESARY WHENEVER THE ROBOT IS MOVING

        movementStartTime = this.time;

        double errorX = globalX - getXPos();
        double errorY = globalY - getYPos();
        double errorH = getErrorRot(globalH);

        currentMovementPower = power;
        targetPosY = globalY;
        targetPosX = globalX;
        targetPosH = globalH;

        double errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
        double errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));

        if (Math.abs(errorH) <= 5) {// attempts to make sure jitters happen less
            errorH = 0; // if error is within 5 degrees on either side we say we're good
        }

        setPower(power, errorXp, errorYp, errorH); // add pid?

    }

    public void moveXY(double x, double y, double power) {
        resetEncoder();

        int adjY = (int) Math.floor((y * inchesToEncoder + 0.5));
        int adjX = (int) Math.floor((x * inchesToEncoder * strafingInefficiencyFactor + 0.5));

        omniMotor0.setTargetPosition(adjY + adjX);
        omniMotor1.setTargetPosition(adjY - adjX);
        omniMotor2.setTargetPosition(adjY - adjX);
        omniMotor3.setTargetPosition(adjY + adjX);

        telemetry.addData("omnimotor 0: ", adjY + adjX);
        telemetry.addData("omnimotor 1: ", adjY - adjX);
        telemetry.addData("omnimotor 2: ", adjY - adjX);
        print("omnimotor 3: ", adjY + adjX);
        setBreakpoint();

        setRunMode();
        setPower(power, adjX, adjY, 0);
    }

    public void encoderRotate(double targetPosRadians, double power, boolean rotateClockwise) {
        // rotates to relative position
        resetEncoder();

        int errorBig = (int) (targetPosRadians * rotationEncoderConstant);

        omniMotor0.setTargetPosition(errorBig * (rotateClockwise ? 1 : -1));
        omniMotor1.setTargetPosition(-errorBig * (rotateClockwise ? 1 : -1));
        omniMotor2.setTargetPosition(errorBig * (rotateClockwise ? 1 : -1));
        omniMotor3.setTargetPosition(-errorBig * (rotateClockwise ? 1 : -1));

        setRunMode();

        setRotateEncoderPower(power, rotateClockwise ? 1 : -1);

        currentPosition = targetPosRadians;

    }

    public void autoRotate(double targetPosDegrees, double power) {
        //direction counter clockwise is -1 clockwise is 1

        double error = getErrorRot(targetPosDegrees);
        boolean isRotating = true;
        double direction = Math.signum(error);
        setRotatePower(power, direction);

        if (Math.abs(error) <= rotationTolerance) {
            print("no rotate needed", "");
            return;
        }
        while (isRotating) {
            updatePosition(); //VERY NESSCESSARY WHENEVER THE ROBOT IS MOVING

            error = getErrorRot(targetPosDegrees);

            if (Math.abs(error) <= rotationTolerance) {
                isRotating = false;
            }
//            else if (Math.abs(error) <= rotationTolerance * 3) {
//                power /= 3;
//            }

            direction = Math.signum(error);
            setRotatePower(power, direction);
        }
        telemetry.addData("rotate end", "");
        telemetry.update();
        setRotatePower(0, 0);
        currentHeading = targetPosDegrees; // updates global heading so we can realign after each movment

    }

    public void autoRotate(double targetPosDegrees, double power, boolean isEncoder) {
        setToRotateRunMode();
        autoRotate(targetPosDegrees, power);
        resetEncoder();

    }


    public double sigmoid(double x) {
        //takes in any x value returns from (0,0) to (1,1) scale x accordingly
        return (2 / (1 + Math.pow(2.71, (-4 * x)))) - 1;
    }

    public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }


    public void setRunMode() {
        omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setToRotateRunMode() {
        omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power, double adjX, double adjY, double adjH) {

        double[] motorPowers = scalePower(
                (adjY + adjX - adjH * rotConst),
                (adjY - adjX + adjH * rotConst),
                (adjY - adjX - adjH * rotConst),
                (adjY + adjX + adjH * rotConst), power);

        omniMotor0.setPower(relativePower(motorPowers[0]));
        omniMotor1.setPower(relativePower(motorPowers[1]));
        omniMotor2.setPower(relativePower(motorPowers[2]));
        omniMotor3.setPower(relativePower(motorPowers[3]));
    }

    public double[] scalePower(double motorPower0, double motorPower1, double motorPower2, double motorPower3, double power) {
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


    public void setRotatePower(double power, double direction) {
        omniMotor0.setPower(relativePower(-direction * power));
        omniMotor1.setPower(relativePower(direction * power));
        omniMotor2.setPower(relativePower(-direction * power));
        omniMotor3.setPower(relativePower(direction * power));
    }

    public void setRotateEncoderPower(double power, double direction) {
        omniMotor0.setPower(relativePower(direction * power));
        omniMotor1.setPower(relativePower(-direction * power));
        omniMotor2.setPower(relativePower(direction * power));
        omniMotor3.setPower(relativePower(-direction * power));
    }


    public void resetEncoder() {
        omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetEncoderNoStop() {
        //WARNING: this uses a deprecated mode (RESET_ENCODERS) may not work as intended
        omniMotor0.setMode(DcMotor.RunMode.RESET_ENCODERS);
        omniMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        omniMotor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        omniMotor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void waitForMotors(double timeout, boolean noPID, double errorBand, boolean noSlowdown) {
        boolean looping = true;
        double errorX = 0;
        double errorY = 0;
        double errorXp;
        double errorYp;
        double errorH;
        double errorHrads;

        if (errorBand == 0) {
            errorBand = acceptableXYError;
        }

        //PID commands
        double movement_pduty = 0;
        double movement_iduty = 0;
        double movement_power;

        Pose2D velocity;


        while (looping) {
            updatePosition(); // VERY NESSCESSARY WHENEVER WE ARE MOVING

            telemetry.addData("heading", getRawHeading());
            telemetry.addData("x: ", errorX);
            print("y: ", errorY);
            errorX = targetPosX - getXPos();
            errorY = targetPosY - getYPos();
            errorH = getErrorRot(targetPosH);
            errorHrads = Math.toRadians(errorH) * 7;

            if (Math.abs(errorH) <= 5) {// attempts to make sure jitters happen less
                errorH = 0; // if error is within 5 degrees on either side we say we're good
            }

            errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
            errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));


            if (noPID) {
                if (getHypotenuse(errorXp, errorYp) < distanceToSlowdown && !noSlowdown) {
                    setPower(slowdownPower, errorXp, errorYp, errorHrads); // add pid?
                    telemetry.addData("slow speed - no pid", "");
                } else {
                    setPower(currentMovementPower, errorXp, errorYp, errorHrads); // add pid?
                    telemetry.addData("full speed - no pid", "");
                }
            } else {
                if (getHypotenuse(errorXp, errorYp) < distanceToSlowdown && !noSlowdown) {
                    setPower(slowdownPower, errorXp, errorYp, errorHrads); // add pid?
                    telemetry.addData("final approach - pid", "");
                } else {

                    movement_pduty = clamp(movement_pgain * Math.pow(getHypotenuse(errorXp, errorYp), 3 / 2), -1, 1);
                    movement_iduty = clamp(movement_igain * (getHypotenuse(errorXp, errorYp)) + movement_iduty, -.7, .7);
                    movement_power = clamp(movement_pduty + movement_iduty, -currentMovementPower, currentMovementPower);
                    setPower(movement_power, errorXp, errorYp, errorHrads);
                    telemetry.addData("current move power: ", movement_power);
                }
            }
            //exit controls
            if (getHypotenuse(errorX, errorY) <= errorBand) {
                looping = false;
            } else if (getHypotenuse(odo.getVelX(), odo.getVelY()) <= minimumXYspeed &&
                    getHypotenuse(errorX, errorY) < acceptableXYError * 4) {
                looping = false;
            } else if ((this.time - movementStartTime) > timeout) {
                looping = false;
            }
            //DASHBOARD TELEMETRY

            velocity = odo.getVelocity();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x pos", getXPos());
            packet.put("y pos", getYPos());
            packet.put("rot pos", getRawHeading());
            packet.put("x vel", velocity.getY(DistanceUnit.INCH));
            packet.put("y vel", velocity.getX(DistanceUnit.INCH));

            dashboard.sendTelemetryPacket(packet);


        }

        currentHeading = getRawHeading();
        setPower(0, 0, 0, 0);

        telemetry.addData("x pos: ", getXPos());
        print("y pos: ", getYPos());

    }

    public void waitForMotors(double timeout, boolean noPid, double errorBand) {
        waitForMotors(timeout, noPid, errorBand, false);
    }

    public void waitForMotors(double timeout, boolean noPID) {
        waitForMotors(timeout, noPID, 0);
    }

    public void waitForMotors(double timeout) {
        waitForMotors(timeout, false, 0);
    }

    public void waitForMotors() {
        waitForMotors(4, false, 0);
    }

    public void waitForMotors(boolean usingJustEncoders) {
        while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy()) {
        }
    }

    public void waitForArm() {
        while (verticalSlide.isBusy()) {
        }
    }

    public void setBreakpoint() {
        while (!gamepad1.a) {
        }
    }

    public double getProportionalSlowdown(double errorX, double errorY) {
        return getHypotenuse(errorX, errorY) * ProportionalCoefficient;
    }

    public double getErrorRot(double targetPosRot) {
        // pos is clockwwise neg is counterclockwise
        return ((targetPosRot - getRawHeading()) + 180) % 360 - 180;
    }

    public void updatePosition() {
        odo.update();
        currentRobotPos = odo.getPosition();
    }

    public double getRawHeading() {
        return currentRobotPos.getHeading(AngleUnit.DEGREES);
    }

    public double getXPos() {
        return currentRobotPos.getX(DistanceUnit.INCH);
    }

    public double getYPos() {
        return currentRobotPos.getY(DistanceUnit.INCH);
    }

    @Override
    public void print(String Name, Object message) {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        telemetry.update();
    }


//    public double getRawHeading() {
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        return pos.h;
//    }
//
//    public double getXPos() {
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        return pos.x;
//    }
//
//    public double getYPos() {
//        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//        return pos.y;
//    }

}
