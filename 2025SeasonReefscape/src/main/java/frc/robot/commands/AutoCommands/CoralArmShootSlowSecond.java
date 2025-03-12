package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralCommands.ShootCoralSlow;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

public class CoralArmShootSlowSecond extends SequentialCommandGroup{
    public CoralArmShootSlowSecond(CoralSubsystem coralSub, AlgaeSubsystem algaeSub, double seconds) {
        addCommands(
            new ParallelRaceGroup(
                new ShootCoralSlow(coralSub, algaeSub),
                new WaitCommand(seconds)   
            )
        );
    }
}
