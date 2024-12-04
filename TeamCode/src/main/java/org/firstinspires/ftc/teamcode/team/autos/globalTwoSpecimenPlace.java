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
        moveToPosition(-2, 31, normalPower);
        waitForMotors();
        waitForArm();

        setVerticalSlide("high chamber place", verticalSlidePower);
        waitForArm();
        setSpecimenWrist("pickup");
        setSpecimenClaw("open");
        setVerticalSlide("low", verticalSlidePower);

        sleep(500);

        moveToPosition(-1, 25, normalPower);
        waitForMotors(2);
        moveToPosition(36, 25, normalPower);
        waitForMotors();
        moveToPosition(36, -3, normalPower);
        waitForMotors(5);

        //pickup specimen from wall
        setSpecimenClaw("closed");
        sleep(250);
        setSpecimenWrist("place");

        moveToPosition(36, 15, normalPower);
        setVerticalSlide("high chamber below", verticalSlidePower / 2);
        waitForMotors(5);
        moveToPosition(-12, 15, normalPower);
        waitForMotors(5);
        moveToPosition(-12, 33, normalPower);
        waitForMotors(2);
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
        waitForMotors(3);
        moveToPosition(40, 47, normalPower);
        waitForMotors(3);
        moveToPosition(40, 0, normalPower);
        waitForMotors(4);
    }
}
