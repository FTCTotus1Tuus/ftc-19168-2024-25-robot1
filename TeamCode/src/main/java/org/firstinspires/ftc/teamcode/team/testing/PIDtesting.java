package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Config
@Autonomous
public class PIDtesting extends DarienOpModeAuto {
    public static double maxPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initControls();

        waitForStart();

        while (opModeIsActive()) {
            moveToPosition(0, 100, maxPower);
            waitForMotors(4);
            setBreakpoint();
            moveToPosition(0, 0, maxPower);
            waitForMotors(4);
            setBreakpoint();
        }

    }
}
