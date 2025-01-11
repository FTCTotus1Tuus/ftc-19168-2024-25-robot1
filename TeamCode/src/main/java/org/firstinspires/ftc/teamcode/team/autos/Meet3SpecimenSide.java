package org.firstinspires.ftc.teamcode.team.autos;
// JMJ

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class Meet3SpecimenSide extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOS.Pose2D currentPosition;

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

        //reset odometry angle (has tendency to drift)
        currentPosition = new SparkFunOTOS.Pose2D(myOtos.getPosition().x, myOtos.getPosition().y, 0);
        myOtos.setPosition(currentPosition);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

        // Pick up one floor samples into observation zone, which will become specimens
        moveToPosition(-2, 25, normalPower); // backup from wall
        waitForMotors(0.5);
        moveToPosition(34.5, 8, 0.7); // go to first sample
        stopIntake();
        setIntakeWrist("down");
        waitForMotors(3);
        startIntake();
        moveToPosition(myOtos.getPosition().x, 24, 0.3);
        waitForMotors(2);
        setIntakeWrist("up");
        moveToPosition(myOtos.getPosition().x, 5, 0.4);
        sleep(500);
        reverseIntake(-0.2);
        waitForMotors(1);
        setBucketPosition("drop"); // drop first sample
        sleep(500);

        moveToPosition(44, 8, 0.7); // go to second sample
        stopIntake();
        waitForMotors(3);
        setBucketPosition("carry");
        setIntakeWrist("down");
        startIntake();
        moveToPosition(myOtos.getPosition().x, 24, 0.3);
        waitForMotors(2);
        setIntakeWrist("up");
        moveToPosition(myOtos.getPosition().x, 4, 0.4);
        sleep(500);
        reverseIntake(-0.2);
        waitForMotors(1);
        stopIntake();
        // rotate to dump sample in corner
        autoRotate(45, .5);
        setBucketPosition("drop"); // drop second sample
        sleep(500);
        autoRotate(0, .5);

        // Specimen 2: Pick up specimen from wall
        moveToPosition(myOtos.getPosition().x, 0, 0.4);
        waitForMotors(2);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-5, 27, 0.6);
        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(3);
        waitForArm();
        moveToPosition(-5, 32, 0.6);
        waitForMotors(1);

        //reset odometry angle (has tendency to drift), assuming we are flush against the sub wall
        currentPosition = new SparkFunOTOS.Pose2D(myOtos.getPosition().x, myOtos.getPosition().y, 0);
        myOtos.setPosition(currentPosition);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 3: Pickup specimen from wall
        moveToPosition(33, 4, 0.6);
        waitForMotors(3);
        moveToPosition(myOtos.getPosition().x, -1, 0.6);
        waitForMotors(1);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-8, 27, 0.6);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(3);
        waitForArm();
        moveToPosition(myOtos.getPosition().x, 31, 0.6);
        waitForMotors(1);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 4: Pickup specimen from wall
        moveToPosition(33, 4, 0.6);
        waitForMotors(3);
        moveToPosition(myOtos.getPosition().x, -1, 0.6);
        waitForMotors(1);
        print("done", "");
        setSpecimenClaw("closed");
        sleep(300);
        setSpecimenWrist("place");

        // Start placing specimens on bar
        moveToPosition(-10, 27, 0.6);
        setVerticalSlide("high chamber below", verticalSlidePower);
        waitForMotors(3);
        waitForArm();
        moveToPosition(myOtos.getPosition().x, 31, 0.6);
        waitForMotors(1);

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenClaw("open");
        setSpecimenWrist("pickup");
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

