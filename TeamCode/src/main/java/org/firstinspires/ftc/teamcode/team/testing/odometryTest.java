package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
public class odometryTest extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();
        waitForStart();

        moveToPosition(10, 0, normalPower);
        waitForMotors();
        print("done 1", "");
        setBreakpoint();
        moveToPosition(0, 10, normalPower);
        waitForMotors();
        print("done 2", "");
        setBreakpoint();
        autoRotate(90, normalPower);
        moveToPosition(0, 0, normalPower);
        waitForMotors();
        print("done 3", "");
        setBreakpoint();
        moveToPosition(5, -5, normalPower);
        waitForMotors();
        print("done 4", "");
        setBreakpoint();
    }

}
