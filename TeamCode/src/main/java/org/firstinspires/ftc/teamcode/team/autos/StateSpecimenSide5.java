package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "State: Specimen Side 5 (R)", group = "State")
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
        moveToPosition(-2, 32, 0.8);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        setSpecimenWrist("place");
        waitForMotors(0.8);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Pick up one floor samples into observation zone, which will become specimens
        moveToPosition(getXPos(), 25, 1); // backup from wall
        waitForMotors(0.5);

        moveToPosition(35.5, 23.5, 1); // go to first sample
        sleep(300);
        readyToPickupSample();
        waitForMotors(); // was 3s
        pickupSample();
        sleep(300);
        moveToPosition(getXPos(), 2, 0.4);
        placeSampleInBucket();
        waitForMotors(2, false, 1);
        setSampleClaw("open");
        sleep(400);

        //sample 2
        moveToPosition(47, 23.5, 1); // go to second sample
        readyToPickupSample();
        waitForMotors();//was 1.2
        pickupSample();
        sleep(100);
        moveToPosition(getXPos(), 2, 0.4);
        placeSampleInBucket();
        waitForMotors(2, false, 1);
        setSampleClaw("open");
        sleep(400);

        //sample 3
        moveToPosition(55, 23.5, 1); // go to third sample

        setIntakeWrist("down");
        setSamplePitch("ready");
        sampleYaw.setPosition(POS_SAMPLE_YAW_CENTER);
        setSampleClaw("openwide");

        waitForMotors(1.5);
        pickupSample();
        sleep(100);
        moveToPosition(getXPos(), 2, 0.4);
        placeSampleInBucket();
        waitForMotors(1.5, false, 0.5);
        setSampleClaw("open");

        moveToPosition(47, 0, 1);
        waitForMotors(1.5);

        setSampleClaw("closed");

        // Specimen 2: Pick up specimen from wall
        setSpecimenClaw("closed"); // keeps the claw slightly open to allow the specimen to fall into position
        sleep(250);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Start placing specimens on bar
        moveToPosition(-4, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2.5, true, 2, true); // was 2.5
        specimenClaw.setPosition(specimenClawClosed - 0.02); // tightens the claw so the specimen doesnt wobble as we clip it
        waitForArm();
        moveToPosition(getXPos(), 31.5, 0.8);
        waitForMotors(1);


        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 3: Pickup specimen from wall
        moveToPosition(33, 6, 1);
        waitForMotors(2.5, true, 2, true); // was 2
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -2, 0.3);
        waitForMotors(0.6); // was 1
        //print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setVerticalSlide("high chamber below", verticalSlidePower);

        // Start placing specimens on bar
        moveToPosition(-5, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        waitForArm();
        moveToPosition(getXPos(), 31.5, 0.8);
        waitForMotors(1);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 4: Pickup specimen from wall
        moveToPosition(33, 8, 1);
        waitForMotors(2, true, 2, true);
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -7, 0.3);
        waitForMotors(0.6);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setVerticalSlide("high chamber below", verticalSlidePower);


        // Start placing specimens on bar
        moveToPosition(-7, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        waitForArm();
        moveToPosition(getXPos(), 31.5, 0.8);
        waitForMotors(1);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 5: Pickup specimen from wall
        moveToPosition(33, 8, 1);
        waitForMotors(2, true, 2, true);
        autoRotate(0, normalPower);
        moveToPosition(getXPos(), -7, 0.3);
        waitForMotors(0.6);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setVerticalSlide("high chamber below", verticalSlidePower);
        // Start placing specimens on bar
        moveToPosition(-10, 27, 1);
        setSpecimenWrist("place");
        waitForMotors(2, true, 2, true);
        specimenClaw.setPosition(specimenClawClosed - 0.02);
        waitForArm();
        moveToPosition(getXPos(), 31.5, 0.8);
        waitForMotors(1, true);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setSamplePitch("arm down");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);
        waitForMotors(2, true, acceptableXYError, true);

    }
}

