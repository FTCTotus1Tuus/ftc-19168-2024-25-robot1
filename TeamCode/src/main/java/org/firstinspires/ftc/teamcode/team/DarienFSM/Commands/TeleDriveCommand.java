package org.firstinspires.ftc.teamcode.team.DarienFSM.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private DoubleSupplier strafe;
    private DoubleSupplier forward;
    private DoubleSupplier turn;

    private GamepadEx drivePad;


    public TeleDriveCommand(DriveSubsystem driveSubsystem, GamepadEx drivePad, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.drivePad = drivePad;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.teleopDrive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble(), drivePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }
}
