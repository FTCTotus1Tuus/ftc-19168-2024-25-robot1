package org.firstinspires.ftc.teamcode.team.DarienFSM.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.team.DarienFSM.DarienRobot;

@TeleOp
public class TestTeleop extends DarienRobot {

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void initialize() {
        super.initialize();
        configureButtons();
    }

    private void configureButtons() {
        //Maps the controller buttons to actions that gamepadEx can preform


        //Uses the mapped buttons to execute commands built in the Robot class
        driveSubsystem.setDefaultCommand(teleDriveCommand);
    }

    @Override
    public void run() {
        super.run();

        
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

    }
}
