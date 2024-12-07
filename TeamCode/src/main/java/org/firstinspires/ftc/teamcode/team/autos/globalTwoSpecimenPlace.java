package org.firstinspires.ftc.teamcode.team.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class globalTwoSpecimenPlace extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        waitForStart();

        setSpecimenClaw("closed");
        setSpecimenWrist("place");
        setVerticalSlide("high chamber below", verticalSlidePower);
        moveToPosition(-2, 31, 0.4);
        waitForMotors(2);
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);

        moveToPosition(-1, 25, normalPower);
        waitForMotors(2);
        moveToPosition(36, 25, 0.4);
        waitForMotors();
        moveToPosition(36, -3, 0.4);
        waitForMotors(3);

        //pickup specimen from wall
        setSpecimenClaw("closed");
        sleep(250);
        setSpecimenWrist("place");

        moveToPosition(36, 15, 0.5);
        setVerticalSlide("high chamber below", verticalSlidePower / 2);
        waitForMotors();
        moveToPosition(-5, 16, 0.6);
        waitForMotors(3);
        moveToPosition(-5, 37, normalPower);
        waitForMotors(1);
        waitForArm();
        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");

        moveToPosition(-7, 25, normalPower);
        setVerticalSlide("low", verticalSlidePower);
        waitForMotors(2);
        moveToPosition(32, 25, normalPower);
        waitForMotors();
        moveToPosition(32, 47, normalPower);
        waitForMotors(2);
        moveToPosition(40, 47, normalPower);
        waitForMotors(1);
        moveToPosition(40, 5, 0.5);
        waitForMotors(4);
    }
}
