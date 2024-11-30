package org.firstinspires.ftc.teamcode.team.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.DarienOpModeTeleop;


@TeleOp
public class touchSensorDebug extends DarienOpModeTeleop {
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        waitForStart();

        while (opModeIsActive()) {
            print("touch Sensor", intakeWristTouchSensor.getValue());
        }
    }
}
