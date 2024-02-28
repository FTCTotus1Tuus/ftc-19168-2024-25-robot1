package org.firstinspires.ftc.teamcode.team;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class DarienOpModeAuto extends DarienOpMode {

    public VisionPortal visionPortal;
    public TeamPropMaskPipeline teamPropMaskPipeline;
    public YellowPixelPlacementPipeline yellowPixelPlacementPipeline;
    public AprilTagProcessor aprilTag;
    double timeRot1=0.5, timeRot2=0.75;

    double timeout = 3; // In seconds
    @Override
    public void runOpMode() throws InterruptedException {}

    public void initControls(boolean isAuto) {
        //isAuto: true=auto false=teleop
        imu = hardwareMap.get(IMU.class, "imu 1");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();

        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor3");
        omniMotor2 = initializeMotor("omniMotor1");
        omniMotor3 = initializeMotor("omniMotor2");

        arm = initializeMotor("arm");

        arm.setDirection(DcMotor.Direction.REVERSE);

        if (isAuto) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            colourSensorLeft = hardwareMap.get(ColorSensor.class, "colourSensorLeft");
            colourSensorRight = hardwareMap.get(ColorSensor.class, "colourSensorRight");

            intakeSensor = hardwareMap.get(ColorSensor.class, "intakeSensor");
        }


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
        //droneLauncher = hardwareMap.get(CRServo.class, "droneLauncher");

    }

    public void initCamera(boolean isBlue) {
        // true = blue false = red
        teamPropMaskPipeline = new TeamPropMaskPipeline(isBlue);
        yellowPixelPlacementPipeline = new YellowPixelPlacementPipeline();
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(teamPropMaskPipeline);
        builder.addProcessor(yellowPixelPlacementPipeline);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()



    public void setWristPosition(String position) {
        switch (position) {
            case "pickup":
                clawWrist.setPosition(clawWristPositionPickup);
                break;
            case "drop":
                clawWrist.setPosition(clawWristPositionDrop);
                break;
            case "dropGround":
                clawWrist.setPosition(clawWristPositionGround);
                break;
            default:
                // do nothing;
        }
    }

    public void setClawPosition(String position) {
        switch (position) {
            case "open":
                clawLeft.setPosition(clawLeftPositionOpen);
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "closed":
                clawLeft.setPosition(clawLeftPositionClosed);
                clawRight.setPosition(clawRightPositionClosed);
                break;
            case "leftOpen":
                clawLeft.setPosition(clawLeftPositionOpen);
                break;
            case "rightOpen":
                clawRight.setPosition(clawRightPositionOpen);
                break;
            case "leftClosed":
                clawLeft.setPosition(clawLeftPositionClosed);
                break;
            case "rightClosed":
                clawRight.setPosition(clawRightPositionClosed);
                break;
            default:
                // do nothing;
        }
    }
        public void setArmPosition(int position, double power) {
        //positive is out
        arm.setTargetPosition(position);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(power);
    }


        public void autoRunMacro(String macro) {
        switch (macro) {
            case "ReadyToPickup":
                setClawPosition("open");

                sleep(50);

                arm.setTargetPosition(0);
                while(arm.isBusy()) {}
                setWristPosition("pickup");
//              changed bc drive doesnt use limit switches
//                  and it broke the arm on the first ready to pickup
//                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                driveArm("in");
//                sleep(200);
//                driveArm("none");

                break;
            case "dropPixel":
                //always put purple pixel to the left
                setClawPosition("open");
                sleep(1000);
                break;
            case "ReadyToDrop":
                setWristPosition("drop");
                break;
            default:
                // do nothing;
                break;
        }
    }

    public boolean isOnLine(boolean isBlue) {
        //|| colourSensorRight.blue() > minBlueVal
        if (isBlue && (colourSensorLeft.blue() > minBlueVal )) {
            return true;
        } else if (!isBlue && (colourSensorLeft.red() > minRedVal)) {
            return true;
        } else {return false;}
    }
    public void backDropPlace(boolean isBlue, int propPosition) {
        if (isBlue) {
            MoveX(-24, 0.5);
            waitForMotors();
            setArmPosition(450, 0.3);
            ArrayList<AprilTagDetection> currentDetections = null;
            double startTime = getRuntime();
            do { currentDetections = aprilTag.getDetections();}
            while (currentDetections.isEmpty() || getRuntime() - startTime > timeout);
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            telemetry.update();
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    alignYellowPixel(detection, propPosition, isBlue, true);
                    return;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            print("timeout","");
            sleep(500);
            switch (propPosition) {
                case 3:
                    MoveX(-23, 0.3);
                    waitForMotors();
                    break;
                case 2:
                    MoveX(-26, 0.3);
                    waitForMotors();
                    break;
                case 1:
                    MoveX(-32, 0.3);
                    waitForMotors();
                    break;
            }
            AutoRotate(90, 0.3, 0);
        }
        else {
            // RED: STRAFE RIGHT TO THE CORRECT POSITION BASED ON THE STRIPE MARK - adding using camera to read as well
            MoveX(27, 0.3);
            waitForMotors();
            setArmPosition(450, 0.3);
            ArrayList<AprilTagDetection> currentDetections = null;
            double startTime = getRuntime();
            print("finding april tag","");
            do { currentDetections = aprilTag.getDetections();}
            while (currentDetections.isEmpty() && (getRuntime() - startTime) < timeout);
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            telemetry.update();
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    alignYellowPixel(detection, propPosition, isBlue, true);
                    return;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            print("timeout","");
            switch (propPosition) { //TODO work on default case
                case 1:
                    MoveX(14.5, 0.3);
                    waitForMotors();
                    break;
                case 2:
                    MoveX(25, 0.3);
                    waitForMotors();
                    break;
                case 3:
                    MoveX(30, 0.3);
                    waitForMotors();
                    break;
            }

    }}

    public void park(boolean isBlue, boolean goInwards, int propPosition) {

        double finalMove = 23;

        StringBuilder builder = new StringBuilder();

        builder.append(isBlue ? "B":"R");
        builder.append(goInwards ? "I":"O");
        builder.append(String.valueOf(propPosition));

        String runSignature = builder.toString();

        print(runSignature,"");
        sleep(1000);

        switch (runSignature){
            case "BI1":
                finalMove = 31; // done
                break;
            case "BI2":
                finalMove = 21;// untested
                break;
            case "BI3":
                finalMove = 15;// untested
                break;
            case "BO1":
                finalMove = -16;// done
                break;
            case "BO2":
                finalMove = -21;// done
                break;
            case "BO3":
                finalMove = -27;// untested
                break;
            case "RI1":
                finalMove = -15;// untested
                break;
            case "RI2":
                finalMove = -21;// untested
                break;
            case "RI3":
                finalMove = -27;// untested
                break;
            case "RO1":
                finalMove = -27;// untested
                break;
            case "RO2":
                finalMove = -21;// untested
                break;
            case "RO3":
                finalMove = -27;// untested
                break;
            default:
                print("sebastian is bad at spelling","");
                return;

        }
            MoveX(finalMove, 0.3);
            waitForMotors();
//        if (goInwards) {
//            if ((isBlue && propPosition==3) || (!isBlue && propPosition==1)) { finalMove-=5;}
//        }
//        else {
//            if ((isBlue && propPosition==1) || (!isBlue && propPosition==3)) { finalMove-=5;}
//        }
//
//
//        finalMove *= isBlue ? 1:-1;
//        finalMove *= goInwards ? 1:-1;
//
//        MoveX(finalMove, 0.3);
//        waitForMotors();

        MoveY(10, 0.1);
        waitForMotors();

    }

    public void alignBackPositions(boolean isBlue, int propPosition){

        ArrayList<AprilTagDetection> currentDetections = null;
        double startTime = getRuntime();
        do { currentDetections = aprilTag.getDetections();}
        while (currentDetections.isEmpty() || getRuntime() - startTime > timeout);
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                alignYellowPixel(detection, propPosition, true, false);
                return;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        print("timeout","");
        sleep(500);

        moveToBackdrop(isBlue);
        autoPlacePixel();
    }

    public void alignYellowPixel(AprilTagDetection tag, int propPosition, Boolean isBlue, boolean isFront) {
//        if (isBlue) {AutoRotate(90, 0.2, 0);}
//        else {AutoRotate(-90, 0.2, 0);} // aligns to the backboard
        double offset=0, robotPositionX;
        print("out of rotate", "");
        int idOffset = 0;
                if (!isBlue) {idOffset = 3;}
        switch (tag.id-idOffset) {
            case 1:
                offset = 0;
                break;
            case 2:
                offset = 6;
                break;
            case 3:
                offset = 12;
                break;
        }
        if (isBlue) {offset++;}
            robotPositionX = -tag.ftcPose.x + offset; //abs pos
            print("", robotPositionX);
        double finalMove = 0;
        switch (propPosition) {
            case 1:
                finalMove = -robotPositionX - 3;
                break;
            case 2:
                finalMove = 6 - robotPositionX - 3;
                break;
            case 3:
                finalMove = 12 - robotPositionX - 3;
                break;
        } // where to go
        if (isBlue) {finalMove -= 0;}

        if (isFront) {finalMove+= 3.5;}

        MoveX(finalMove, 0.3);
        waitForMotors();

        MoveY(4, 0.1);
        waitForMotors();

        telemetry.addData("",yellowPixelPlacementPipeline.getPixelCount());
        print(yellowPixelPlacementPipeline.isOnLeft() ? "left":"right","");
        if (isFront && !yellowPixelPlacementPipeline.isOnLeft()) {
            if (isBlue) {MoveX(-4.75, 0.1);}
            else {MoveX(-6.5, 0.1);}
        } else {
            MoveX(-2.25, 0.1);

            setArmPosition(775, 0.1);
                }
            setClawPosition("leftOpen");
            setWristPosition("drop");
            sleep(250);
        waitForMotors();

        MoveY(tag.ftcPose.y-5.5, 0.1);
        waitForMotors();

        autoPlacePixel();

    }

    public void moveToBackdrop(boolean isBlue) {
        MoveY(24, 0.2);
        while (!isOnLine(isBlue)) {}
        print("found line", "");
        MoveY(3,0.1); // chagne to 2.5
        waitForMotors();
    }

    public void autoPlacePixel() {
        waitForMotors();
        setClawPosition("rightOpen");
        sleep(150);
        MoveY(-4.5, 0.25);
        waitForMotors();
        setWristPosition("dropGround");
        return;
    }
    public void pickUpWhite() {
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
        feeder.setPower(-1);
        waitForMotors();
        sleep(1250);

    }

    public void MoveY(double y, double power) {

        resetEncoder();
        encoderPos = (int) Math.floor((y * constant + 0.5));
        setTargetPosY();
        print("moving y",encoderPos);
        setRunMode();

        setPower(power);
    }


    public void MoveX(double x, double power) {
        resetEncoder();

        encoderPos = (int) Math.floor((x * constant * 1.111111) + 0.5);
        print("moving x",encoderPos);
        setTargetPosX();
        setRunMode();
        setPower(power);
    }

    public void AutoRotate(double TargetPosDegrees, double power, int direction) {
        //direction counter clockwise is -1 clockwise is 1
        double timeoutS = 3;
        rotationTolerance = 5;


        setToRotateRunMode();
        double truePower;
        double error;
        boolean isRotating = true;
        if ( !(Math.abs(TargetPosDegrees-getRawHeading()) < rotationTolerance)){
            if (direction == 0) {
                // rotate the shortest way.
                if ((TargetPosDegrees-getRawHeading()) > 0) {direction=-1;}
                else {direction = 1;}
                rotationTolerance = 2;
                power/=2;
            }
            double startTime = getRuntime();
            setRotatePower(power, direction);
                while (isRotating) {
                    error = Math.abs(TargetPosDegrees-getRawHeading());
    //              truePower = Math.min(Math.pow((error/scaleConstant),2),1);
                    telemetry.addData("heading ", getRawHeading());
                    telemetry.addData("error ", error);
                    print("power", power);

                   if (error<=rotationTolerance) {
                        isRotating = false;
                    } else if (getRuntime() - startTime > timeoutS) {
                        isRotating = false;
                        telemetry.addData("timeout","");
                    }
        }}
        telemetry.addData("rotate end", "");
        telemetry.update();
        setRotatePower(0,0);
        resetEncoder();
    }

    public int getPropPosition() {
        double[] results = teamPropMaskPipeline.getLastResults();
        if (results[0] > results[1] && results[0] > results[2]) {return 1;}
        else if (results[1] > results[0] && results[1] > results[2]) {return 2;}
        else {return 3;}
    }
    public int getPropPositionRR(SampleMecanumDrive drive) {
        final double[] firstResults = new double[1];
        final double[] secondResults1 = new double[1];
        final double[] thirdResults = new double[1];
        final double[] secondResults2 = new double[1];


        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(2).build());

        drive.turn(Math.toRadians(5));
            firstResults[0] = teamPropMaskPipeline.getLastResults()[0];
            secondResults1[0] = teamPropMaskPipeline.getLastResults()[1];
        drive.turn(Math.toRadians(-10));
            thirdResults[0] = teamPropMaskPipeline.getLastResults()[2];
            secondResults2[0] = teamPropMaskPipeline.getLastResults()[1];

            drive.turn(Math.toRadians(5));


        double secondResults = (secondResults1[0] +secondResults2[0])/2;


        if (firstResults[0] > secondResults && firstResults[0] > thirdResults[0]) {return 1;}
        else if (secondResults > firstResults[0] && secondResults > thirdResults[0]) {return 2;}
        else {return 3;}



    }

        public double sigmoid(double x) {
        //takes in any x value returns from (0,0) to (1,1) scale x accordingly
        return (2/(1+Math.pow(2.71,(-4*x))))-1;
        }
        public double getVoltage () {
            return (hardwareMap.voltageSensor.iterator().next().getVoltage());
        }


        public void setTargetPosY ()
        {

            omniMotor0.setTargetPosition(encoderPos);
            omniMotor1.setTargetPosition(encoderPos);
            omniMotor2.setTargetPosition(-encoderPos);
            omniMotor3.setTargetPosition(-encoderPos);

        }

        public void setTargetPosX ()
        {

            omniMotor0.setTargetPosition(encoderPos);
            omniMotor1.setTargetPosition(-encoderPos);
            omniMotor2.setTargetPosition(encoderPos);
            omniMotor3.setTargetPosition(-encoderPos);

        }
        public void setRunMode ()
        {
            omniMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            omniMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            omniMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            omniMotor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public void setToRotateRunMode () {
            omniMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            omniMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            omniMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            omniMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        public void setPower ( double power)
        {
            omniMotor0.setPower(relativePower(power));
            omniMotor1.setPower(relativePower(power));
            omniMotor2.setPower(relativePower(power));
            omniMotor3.setPower(relativePower(power));

        }

        public void setRotatePower ( double power, double direction){
            omniMotor0.setPower(relativePower(direction * power));
            omniMotor1.setPower(relativePower(-direction * power));
            omniMotor2.setPower(relativePower(-direction * power));
            omniMotor3.setPower(relativePower(direction * power));

        }

        public void resetEncoder ()
        {
            omniMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void waitForMotors ()
        {
            while (omniMotor0.isBusy() && omniMotor1.isBusy() && omniMotor2.isBusy() && omniMotor3.isBusy()) {
            }
        }
        public double getRawHeading () {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

    }