package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class Meet3SpecimenSide extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        waitForStart();
        setBucketPosition("carry");

        // Specimen 1
        moveToPosition(-2, 31, 0.4);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(2);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);

        // Push one floor samples into observation zone, which will become specimens
        moveToPosition(-2, 25, normalPower); // backup from wall
        waitForMotors(0.5);
        moveToPosition(36, 8, 0.7); // go to first sample
        waitForMotors(2);
        setIntakeWrist("down");
        startIntake();
        moveToPosition(36, 23, 0.3);
        waitForMotors(1);
        setIntakeWrist("up");
        moveToPosition(39, 5, 0.4);
        sleep(500);
        reverseIntake();
        waitForMotors(1);
        setBucketPosition("drop"); // drop first sample
        sleep(500);

        moveToPosition(44, 8, 0.5); // go to second sample
        stopIntake();
        waitForMotors();
        setBucketPosition("carry");
        setIntakeWrist("down");
        startIntake();
        moveToPosition(44, 24, 0.2);
        waitForMotors(2);
        setIntakeWrist("up");
        moveToPosition(48, 4, 0.3);
        sleep(500);
        reverseIntake();
        waitForMotors(1);
        setBucketPosition("drop"); // drop second sample
        sleep(500);

//        moveToPosition(56, 8, 0.5); // go to third sample
//        stopIntake();
//        waitForMotors(0.75);
//        setBucketPosition("carry");
//        setIntakeWrist("down");
//        startIntake();
//        moveToPosition(56, 30, 0.3);
//        waitForMotors(1);
//        setIntakeWrist("up");
//        moveToPosition(35, 4, 0.3);
//        sleep(500);
//        reverseIntake();
//        waitForMotors(1);
//        setBucketPosition("drop"); // drop second sample
//        setSpecimenWrist("pickup");
//        sleep(1000);
//        setBucketPosition("carry");
//        stopIntake();

// Specimen 2: Pick up specimen from wall
        moveToPosition(32, -4, 0.4);
        waitForMotors();
        print("done", "");
        setSpecimenClaw("closed");
        sleep(150);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-5, 31, 0.6);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(3);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);


        moveToPosition(32, -4, 0.4);
        waitForMotors();
        print("done", "");
        setSpecimenClaw("closed");
        sleep(150);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-5, 31, 0.6);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(3);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);


        moveToPosition(32, -4, 0.4);
        waitForMotors();
        print("done", "");
        setSpecimenClaw("closed");
        sleep(150);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-10, 31, 0.6);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(3);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);

        // PARK: Go to observation zone.
        moveToPosition(40, 5, 1);

        double errorX;
        double errorY;
        double errorXp;
        double errorYp;
        double errorH;

        while (opModeIsActive()) {
            errorX = 40 - getXPos();
            errorY = 5 - getYPos();
            errorH = getErrorRot(currentHeading);
            errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
            errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));

            setPower(0.6, errorXp, errorYp, errorH); // add pid?
        }
    }
}

