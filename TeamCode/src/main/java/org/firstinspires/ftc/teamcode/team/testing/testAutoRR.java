package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class testAutoRR extends DarienOpModeAuto {
        @Override
        public void runOpMode() {
            Pose2d startPos = new Pose2d(-36, -64, Math.toRadians(90.00));

            initControlsRR(true);
            initCamera(false);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//            drive.setPoseEstimate(startPos);

//            drive.setPoseEstimate(new Pose2d(-36.04, -61.37, Math.toRadians(90.00)));



//            Trajectory leftTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();
//            Trajectory middleTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();
//            Trajectory rightTrajectory = drive.trajectoryBuilder(new Pose2d(-36.04, -61.37, Math.toRadians(90.00))).build();

//            Trajectory centerOnTile = drive.trajectoryBuilder(new Pose2d())
//                    .forward(24)
//                    .addDisplacementMarker(10, () -> {
//                        setArmPosition(250, 0.3);
//                        setWristPosition("dropGround");
//                    })
//                    .build();
//            Trajectory propPositionOne = drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .strafeRight(24).build(); //25 TODO fix this traj prob messing up pos 1
//            Trajectory propPositionTwo = drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .strafeLeft(24).build();  //20
//            Trajectory forwardSixteen = drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .forward(16).build();


            TrajectorySequence middle = drive.trajectorySequenceBuilder(startPos)
                    .splineTo(new Vector2d(-35.89, -38.5), Math.toRadians(90.00))
                    .addDisplacementMarker(() -> {
                        autoRunMacro("dropPixel");
                    })
                    .lineTo(new Vector2d(-48, -39))
                    .addDisplacementMarker(() -> {
                        autoRunMacro("ReadyToPickup");
                        setClawPosition("closed");
                    })
                    .strafeLeft(2)
                    .forward(3)
                    .addDisplacementMarker(() -> {
                      drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().vec(), Math.toRadians(90)));
                    })
                    .splineTo(new Vector2d(-43, -12), Math.toRadians(90))
                    .lineTo(new Vector2d(28, -12))
                    .addDisplacementMarker(() -> {
                        setArmPosition(250, 0.1);
                        autoRunMacro("ReadyToDrop");
                    })
                    .waitSeconds(1)
                    .splineTo(new Vector2d(43, -34), Math.toRadians(0))
                    .addDisplacementMarker(() -> {
                        setArmPosition(0, 0.3);
                    })
                    .forward(10,  SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).addDisplacementMarker(() -> {
                        autoRunMacro("dropPixel");
                        setWristPosition("pickup");
                    })
                    .back(4)
                    .strafeLeft(30)
                    .forward(12)
                    .build();
            TrajectorySequence readyPlaceOne = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(0, () -> {
                        while (!isOnLine(false)) {}
                        drive.breakFollowing();
                    })
                    .forward(24)
                    .build();
            TrajectorySequence readyPlaceTwo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(4,  SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(30)
                    .forward(12)
                    .build();


            waitForStart();

            if(isStopRequested()) return;

            int propPosition = getPropPositionRR(drive);
            drive.setPoseEstimate(startPos);

            autoRunMacro("ReadyToPickup");
            setClawPosition("leftClosed"); // makes sure that the purple pixel is picked up
            setArmPosition(250, 0.3);
            while(leftArm.isBusy()) {}
            setWristPosition("dropGround");
            sleep(333);
            print("prop position", propPosition);
            //should be centered on the spike mark tile
            switch (propPosition) {
                case 1:
//                    drive.turn(Math.toRadians(90)); // turns to spike mark
//                    autoRunMacro("dropPixel"); // places the purple pixel on the ground
//                    drive.followTrajectory(propPositionOne);  // moves 1 tile right to be facing the backdrop
//                    autoRunMacro("ReadyToPickup"); // returns the wrist
//                    drive.turn(180);
                    break;
                case 2:
                    drive.followTrajectorySequence(middle); // up to in front of backboard
//                    drive.followTrajectorySequence(readyPlaceOne);
//                    drive.followTrajectorySequence(readyPlaceTwo);
//                    autoRunMacro("dropPixel");
//                    drive.followTrajectorySequence(park);

                    break;
                case 3:
//                    drive.turn(Math.toRadians(-90)); // turns to spike mark
//                    autoRunMacro("dropPixel"); // places the pixel
//                    autoRunMacro("ReadyToPickup"); // returns the wrist
//                   drive.followTrajectory(propPositionTwo); // strafe left to the center of the tile, facing the backdrop
                    break;
            }
//            drive.followTrajectory(mainTrajectory);

        }
    }

