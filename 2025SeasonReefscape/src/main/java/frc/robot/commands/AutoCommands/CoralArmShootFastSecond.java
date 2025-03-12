package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralCommands.ShootCoralFast;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

public class CoralArmShootFastSecond extends SequentialCommandGroup{
    public CoralArmShootFastSecond(CoralSubsystem coralSub, AlgaeSubsystem algaeSub, double seconds) {
        addCommands(
            new ParallelRaceGroup(
                new ShootCoralFast(coralSub, algaeSub),
                new WaitCommand(seconds)   
            )
        );
    }
}
