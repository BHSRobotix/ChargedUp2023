package bhs.devilbotz.commands.auto;

import bhs.devilbotz.commands.Balance;
import bhs.devilbotz.commands.Forward;
import bhs.devilbotz.commands.Turn;
import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BalanceAuto extends SequentialCommandGroup {
    private final DriveTrain drive;
    public BalanceAuto(DriveTrain drive) {
        this.drive = drive;
        addRequirements(drive);

        addCommands(
                new Forward(drive),
                new Balance(drive),
                new WaitCommand(2),
                new Turn(drive)
        );

    }
}