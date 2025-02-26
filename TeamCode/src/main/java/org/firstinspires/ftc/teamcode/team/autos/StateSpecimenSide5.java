package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "State: Specimen Side (R)", group = "State")
public class StateSpecimenSide5 extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        setBucketPosition("up");
        setSamplePitch("arm down");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        waitForStart();
        setIntakeSlidePower(-0.05);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Specimen 1
        moveToPosition(-2, 31, 0.8);
        specimenClaw.setPosition(specimenClawClosed - 0.03);
        setSpecimenWrist("place");
        waitForMotors(0.8);
        waitForArm();
        sleep(50);
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Pick up one floor samples into observation zone, which will become specimens
        updatePosition();
        moveToPosition(getXPos(), 25, 1); // backup from wall
        waitForMotors(0.5);

        moveToPosition(35.5, 23.5, 1); // go to first sample
        sleep(300);
        readyToPickupSample();
        waitForMotors(); // was 3s
        pickupSample();
        sleep(100);
        updatePosition();
        moveToPosition(getXPos(), 6, 0.9); // Set Y target to a few inches from wall to avoid slamming into the wall as robot skids into position.
        placeSampleInBucket();
        sleep(400);
        setSampleClaw("open");
        waitForMotors(1.5, false, 1);

        //sample 2
        moveToPosition(45.5, 23.5, 1); // go to second sample
        readyToPickupSample();
        waitForMotors();//was 1.2
        pickupSample();
        sleep(100);
        updatePosition();
        moveToPosition(getXPos(), 6, 0.9); // Set Y target to a few inches from wall to avoid slamming into the wall as robot skids into position.
        placeSampleInBucket();
        sleep(400);
        setSampleClaw("open");
        waitForMotors(1.5, false, 1);

        //sample 3
        moveToPosition(55, 23.5, 1); // go to third sample

        setIntakeWrist("down");
        setSamplePitch("ready");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        setSampleClaw("open");

        waitForMotors(1.5);
        pickupSample();
        sleep(100);
        moveToPosition(40, -2, 1); // Set a negative Y position to reach the wall.
        placeSampleInBucket();
        waitForMotors(1.5, false, 0.5);
        setSampleClaw("open");
        sleep(100);

        // Specimen 2: Pick up specimen from wall
        setSpecimenClaw("closed"); // keeps the claw slightly open to allow the specimen to fall into position
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Place specimen on chamber
        moveToPosition(-4, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true); // was 2.5
        specimenClaw.setPosition(specimenClawClosed - 0.02); // tightens the claw so the specimen doesnt wobble as we clip it
//        waitForArm(); TODO possibly risky
//        updatePosition();
//        moveToPosition(getXPos() - 3, 31, 0.8);
//        waitForMotors(0.5, true, 0.25, true);
        setPower(0.3, 0, 1, 0);
        sleep(150);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 3: Pickup specimen from wall
        moveToPosition(33, 3, 1);
        waitForMotors(2.5, true, 2, true); // was 2
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -2, 0.5);
        waitForMotors(0.3);
        //print("done", "");
        setSpecimenClaw("closed");
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Place specimen on chamber
        moveToPosition(-6, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
//        waitForArm(); TODO
//        moveToPosition(getXPos() - 3, 31, 0.8);
//        waitForMotors(0.5, true, 0.25, true);

        setPower(0.3, 0, 1, 0);
        sleep(150);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);


        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 4: Pickup specimen from wall
        moveToPosition(33, 3, 1);
        waitForMotors(2, true, 2, true);
        autoRotate(0, normalPower);
        updatePosition();
        moveToPosition(getXPos(), -7, 0.5);
        waitForMotors(0.3);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);


        // Place specimen on chamber
        moveToPosition(-9, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
//        waitForArm(); TODO
//        updatePosition();
//        moveToPosition(getXPos() - 3, 31, 0.8);
//        waitForMotors(0.5, true, 0.25, true);

        setPower(0.3, 0, 1, 0);
        sleep(150);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);

        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 5: Pickup specimen from wall
        moveToPosition(33, 3, 1);
        waitForMotors(2, true, 2, true);
        autoRotate(0, normalPower);
        updatePosition();
        moveToPosition(getXPos(), -7, 0.5);
        waitForMotors(0.3);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(200);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Place specimen on chamber
        moveToPosition(-12, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(1.8, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
//        waitForArm(); TODO
//        updatePosition();
//        moveToPosition(getXPos() - 3, 31, 0.8);
//        waitForMotors(0.5, true, 0.25, true);

        setPower(0.3, 0, 1, 0);
        sleep(150);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setPower(0, 0, 0, 0);

        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);

        double errorX = 0;
        double errorY = 0;
        double errorXp;
        double errorYp;
        double errorH;
        double errorHrads;

        while (opModeIsActive()) {
            errorX = targetPosX - getXPos();
            errorY = targetPosY - getYPos();
            errorH = getErrorRot(targetPosH);
            errorHrads = Math.toRadians(errorH) * 7;

            errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
            errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));


            setPower(slowdownPower, errorXp, errorYp, errorHrads); // add pid?
        }

    }
}

