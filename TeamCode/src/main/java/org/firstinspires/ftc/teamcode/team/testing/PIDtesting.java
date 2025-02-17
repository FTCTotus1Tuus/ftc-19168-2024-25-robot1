package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Config
@Autonomous
public class PIDtesting extends DarienOpModeAuto {
    public static double maxPower = 0;
    public static double xTarget = 0;
    public static double yTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initControls();

        waitForStart();

        while (opModeIsActive()) {
            moveToPosition(xTarget, yTarget, maxPower);
            waitForMotors(4);

            setBreakpoint();
        }

    }
}
