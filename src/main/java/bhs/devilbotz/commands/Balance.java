package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {
    private final DriveTrain drive;

    private boolean done = false;
    private final AHRS navx;
    /**
     * Creates a new Balance.
     */
    public Balance(DriveTrain drive) {
        this.drive = drive;
        navx = drive.getNavx();
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Pitch (Side to side): " + navx.getPitch());
        System.out.println("Roll (Front to Back): " + navx.getRoll());
        System.out.println("Yaw (Rotation): " + navx.getYaw());

        if (navx.getRoll() > 8) {
            System.out.println(">5");
            drive.arcadeDrive(-0.4, 0);
            done = false;
        } else if (navx.getRoll() < -8) {
            System.out.println("<5");
            drive.arcadeDrive(0.4, 0);
            done = false;
        } else {
            done = true;
            drive.arcadeDrive(0, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    Timer timer = new Timer();

    public boolean isTrueForTwoSeconds(boolean input) {
        if (input) {
            System.out.println("Timer Started");
            timer.start();
        } else {
            System.out.println("Timer reset");
            timer.stop();
            timer.reset();
        }
        return timer.get() >= 2.0;
    }
    @Override
    public boolean isFinished() {
        // if the navx roll is true for 2+ seconds, then return true
        return isTrueForTwoSeconds(done);
    }
}
