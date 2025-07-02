package org.firstinspires.ftc.teamcode.team.DarienFSM.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.team.DarienFSM.Subsystems.DriveSubsystem;

public class AutoDriveToCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private double x, y, rot, maxPower;
    private boolean noPid, noSlowdown;

    public AutoDriveToCommand(DriveSubsystem driveSubsystem, double x, double y, double rot, boolean noPID, boolean noSlowdown, double maxPower) {
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.noPid = noPID;
        this.noSlowdown = noSlowdown;
        this.maxPower = maxPower;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() { // TODO SOLVERS
        driveSubsystem.setAutoDestination(x, y, rot, noPid, noSlowdown, maxPower);
    }

    @Override
    public void execute() { // TODO SOLVERS
        driveSubsystem.autoUpdatePower();
    }

    @Override
    public boolean isFinished() { // TODO SOLVERS
        return driveSubsystem.isMovementDone();

    }
}
