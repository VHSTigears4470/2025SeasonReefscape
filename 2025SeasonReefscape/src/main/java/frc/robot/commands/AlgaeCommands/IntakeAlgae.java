package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAlgae extends Command {
    private final AlgaeSubsystem m_algaeSub;
    private final IntakeSubsystem m_intakeSub;

    public IntakeAlgae(AlgaeSubsystem algaeSub) {
        m_algaeSub = algaeSub;
        addRequirements(algaeSub);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time he scheduler runs while the command is scheduled.
    // Probably need to put something here
    @Override
    public void execute() {
        m_algaeSub.intake();
    } 
    
    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //probably stop motors or something
    m_algaeSub.stop();
  }

  // Returns true when the command should end.
  //maybe need to change
  @Override
  public boolean isFinished() {
    return false;
  }
}
