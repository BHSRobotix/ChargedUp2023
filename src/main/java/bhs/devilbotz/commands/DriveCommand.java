package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveTrain drive;
    private final DoubleSupplier speed;
    private final DoubleSupplier rotation;

    private final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(3);

    private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(3);

    public DriveCommand(DriveTrain drive, DoubleSupplier speed, DoubleSupplier rotation) {
        this.drive = drive;
        this.speed = speed;
        this.rotation = rotation;
        addRequirements(this.drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.arcadeDrive(
                speedSlewRateLimiter.calculate(-speed.getAsDouble()),
                rotationSlewRateLimiter.calculate(-rotation.getAsDouble()
                ));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
