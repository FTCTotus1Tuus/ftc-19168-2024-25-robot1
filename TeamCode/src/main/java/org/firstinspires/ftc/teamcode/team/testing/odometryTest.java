package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous
@Disabled
public class odometryTest extends DarienOpModeAuto {
    public void runOpMode() {
        initControls();
        waitForStart();

        moveToPosition(10, 0, normalPower);
        waitForMotors();
        setBreakpoint();
        moveToPosition(0, 10, normalPower);
        waitForMotors();

    }
}
