package org.firstinspires.ftc.teamcode.team.DarienFSM.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private DoubleSupplier strafe;
    private DoubleSupplier forward;
    private DoubleSupplier turn;

    private GamepadEx drivePad;


    public TeleDriveCommand(DriveSubsystem driveSubsystem, GamepadEx drivePad, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.drivePad = drivePad;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() { // TODO SOLVERS
        driveSubsystem.teleDrive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble(), drivePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }
}
