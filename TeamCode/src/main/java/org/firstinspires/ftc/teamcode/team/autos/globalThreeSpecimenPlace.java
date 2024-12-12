package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class globalThreeSpecimenPlace extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        waitForStart();

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

        // Push one floor sample into observation zone, which will become specimen #3.
        moveToPosition(-2, 25, normalPower);
        waitForMotors(0.5);
        moveToPosition(30, 25, 0.4);
        waitForMotors();
        moveToPosition(30, 47, 0.4);
        waitForMotors(1.5);
        moveToPosition(40, 47, 0.4);
        waitForMotors(1);
        moveToPosition(40, 6, 0.7);
        waitForMotors(2);
        moveToPosition(32, 4, 0.4);
        waitForMotors(1.5);
        moveToPosition(32, -3, 0.4);
        waitForMotors(1);

        // Specimen 2: Pick up specimen from wall
        setSpecimenClaw("closed");
        sleep(150);
        setSpecimenWrist("place");

        // Go to the sub to score the specimen
        setVerticalSlide("high chamber below", verticalSlidePower / 2);
        moveToPosition(-5, 16, 0.8);
        waitForMotors(2);
        moveToPosition(-5, 39, normalPower);
        waitForMotors(1);
        waitForArm();
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);

        // Specimen 3
        // Go to observation zone to pick up specimen from wall
        moveToPosition(-1, 25, normalPower);
        waitForMotors(0.5);
        moveToPosition(36, 25, 0.4);
        waitForMotors();
        moveToPosition(36, -3, 0.5);
        waitForMotors(1.5);

        // Specimen 3: Pickup specimen from wall
        setSpecimenClaw("closed");
        sleep(150);
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower / 2);


        moveToPosition(-8, 16, 0.8);
        waitForMotors(2);
        moveToPosition(-8, 40, 0.4);
        waitForMotors(1);
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
        
        while (opModeIsActive()) {
            errorX = 40 - getXPos();
            errorY = 5 - getYPos();
            errorXp = (errorX * Math.cos(Math.toRadians(getRawHeading()))) + errorY * Math.sin(Math.toRadians(getRawHeading()));
            errorYp = (-errorX * Math.sin(Math.toRadians(getRawHeading()))) + errorY * Math.cos(Math.toRadians(getRawHeading()));

            if (getHypotenuse(errorXp, errorYp) > 5) {
                setPower(currentMovementPower, errorXp, errorYp); // add pid?
            } else {
                setPower(currentMovementPower / 2, errorXp, errorYp); // add pid?
            }
        }
    }

}

