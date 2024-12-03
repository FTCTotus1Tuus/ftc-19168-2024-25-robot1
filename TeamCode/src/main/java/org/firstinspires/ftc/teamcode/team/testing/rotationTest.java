package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Config
@Autonomous
public class rotationTest extends DarienOpModeAuto {

    public static double rotateAmount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        waitForStart();

        while (opModeIsActive()) {
            autoRotate(rotateAmount, normalPower);
            setBreakpoint();
        }
    }
}
