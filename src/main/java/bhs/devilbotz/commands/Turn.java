package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Turn extends CommandBase {
    private final DriveTrain drive;
    private final AHRS navx;
    /**
     * Creates a new Balance.
     */
    public Turn(DriveTrain drive) {
        this.drive = drive;
        navx = drive.getNavx();
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        navx.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.arcadeDrive(0, 0.4);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return navx.getYaw() >= 45;
    }
}
