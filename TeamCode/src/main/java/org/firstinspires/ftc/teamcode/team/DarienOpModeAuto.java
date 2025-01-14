package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.apache.commons.math3.geometry.euclidean.twod.Line;


@Config
public class DarienOpModeAuto extends DarienOpMode {

    public static double normalPower = 0.3;
    public static double verticalSlidePower = 1; //swapped to 1 from 0.8 needs testing
    public static double strafingInefficiencyFactor = 1.145;

    //vertical slide positions
    public static int highChamberBelowPos = 1600;
    public static int highChamberPlacePos = 2300;
    public static int barBelow2Pos;
    public static int barPlace2Pos;
    public static int barBelow1Pos;
    public static int barPlace1Pos = 950; // TODO fix value random guess made in the delerium of exhaustion
    public static int basketLowPos = 2450;
    public static int basketHighPos = 4380;
    public static int armGroundPos = 0;

    //encoder movement targets
    public double targetPosX = 0;
    public double targetPosY = 0;
    public double targetPosH = 0;
    public double rotConst = 1;
    public double acceptableXYError = 0.5; //how many inches off the xy movement can be - does not compound
    public static double minimumXYspeed = 0.5;
    public double currentMovementPower = 0;
    public static double ProportionalCoefficient = 0.3;

    public double currentHeading = 0;

    public double specimenWristUp = 0.48;

    public double currentPosition = 0;
    public static double rotationEncoderConstant = 557;

    @Override
    public void initControls() {
        super.initControls();

        print("otos", myOtos);

        // reverse motors 2 and 3
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);

        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this is key


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
                break;
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
                break;

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
                break;

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
                break;
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
        }
    }

    public void startIntake() {
        intakeWheels.setPower(powerIntakeWheelToPickupSample);
    }

    public void stopIntake() {
        intakeWheels.setPower(0);
    }

    public void reverseIntake() {
        intakeWheels.setPower(-0.5);
    }

    public void reverseIntake(double power) {
        intakeWheels.setPower(power);
    }

    public void placeSampleInBucket() {
        intakeSlide.setPower(-0.5);
        setIntakeWrist("up");
        setBucketPosition("carry");
        sleep(2000); // TODO change to set to how long for intake slide to go in
        reverseIntake();
        sleep(500);
        stopIntake();

    }


    public void moveToPosition(double globalX, double globalY, double power) {
        moveToPosition(globalX, globalY, currentHeading, power);
    }

    public void moveToPosition(double globalX, double globalY, double globalH, double power) {
        // uses optical sensor to move by setting robot motor power
        // DOES NOT USE ENCODERS
        double errorX = globalX - getXPos();
        double errorY = globalY - getYPos();
        double errorH = getErrorRot(globalH);

        currentMovementPower = power;
        targetPosY = globalY;
        targetPosX = globalX;
        targetPosH = globalH;

        double errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
        double errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));

        if (Math.abs(errorH) <= 2.5) {// attempts to make sure jitters happen less
            errorH = 0; // if error is within 2.5 degrees on either side we say we're good
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

    public void waitForMotors(double timeout) {
        boolean looping = true;
        double errorX = 0;
        double errorY = 0;
        double errorXp;
        double errorYp;
        double errorH;
        double startTime = this.time;
        while (looping) {
            telemetry.addData("x: ", errorX);
            print("y: ", errorY);
            errorX = targetPosX - getXPos();
            errorY = targetPosY - getYPos();
            errorH = getErrorRot(targetPosH);

            if (Math.abs(errorH) <= 2.5) {// attempts to make sure jitters happen less
                errorH = 0; // if error is within 2.5 degrees on either side we say we're good
            }

            errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
            errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));

            if (getHypotenuse(errorXp, errorYp, errorH) > 5) {
                setPower(currentMovementPower, errorXp, errorYp, errorH); // add pid?
            } else {
                setPower(currentMovementPower / 2, errorXp, errorYp, errorH); // add pid?
            }
            telemetry.addData("x vel: ", myOtos.getVelocity().x);
            telemetry.addData("y vel: ", myOtos.getVelocity().y);

            if (getHypotenuse(errorX, errorY, errorH) <= acceptableXYError) {
                looping = false;
            } else if (getHypotenuse(myOtos.getVelocity().x, myOtos.getVelocity().y, myOtos.getVelocity().h) <= minimumXYspeed &&
                    getHypotenuse(errorX, errorY, errorH) < acceptableXYError * 4) {
                looping = false;
            } else if ((this.time - startTime) > timeout) {
                looping = false;
            }
        }

        currentHeading = getRawHeading();
        setPower(0, 0, 0, 0);

        telemetry.addData("x pos: ", getXPos());
        print("y pos: ", getYPos());

    }

    public void waitForMotors() {
        waitForMotors(4);
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


    public double getRawHeading() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return pos.h;
    }

    public double getXPos() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return pos.x;
    }

    public double getYPos() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return pos.y;
    }

}
