package org.firstinspires.ftc.teamcode.team.autos;

import androidx.collection.CircularArray;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Config
@Autonomous
public class AutoTesting extends DarienOpModeAuto {

        public static double servoPosition = 0;

    public void runOpMode() {
        Servo bucketServo = hardwareMap.get(Servo.class, "bucket");
        Servo specimenWrist = hardwareMap.get(Servo.class, "specimenWrist");
        Servo intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        Servo specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        waitForStart();
//        bucketServo.setPosition(0);

//        specimenWrist.setPosition(0);

        while(opModeIsActive()) {
            if (gamepad1.a) {
                specimenClaw.setPosition(servoPosition);
            }
        }

    }

}
