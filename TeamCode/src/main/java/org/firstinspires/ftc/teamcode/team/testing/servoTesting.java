package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.DarienOpModeTeleop;

@TeleOp
@Disabled
public class servoTesting extends DarienOpModeTeleop {
    @Override
    public void runOpMode() throws InterruptedException {
        if (gamepad2.right_trigger > 0.5) {
            specimenClaw.setPosition(specimenClawOpen);
        } else {
            specimenClaw.setPosition(specimenClawClosed);
        }
    }
}
