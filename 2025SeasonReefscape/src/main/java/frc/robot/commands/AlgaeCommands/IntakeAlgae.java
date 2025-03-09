package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class IntakeAlgae extends Command {
    private final AlgaeSubsystem m_algaeSub;

    //Constructor for IntakeAlgae, also adds requirments so that this is the only command using algaeSub.
    public IntakeAlgae(AlgaeSubsystem algaeSub) {
        m_algaeSub = algaeSub;
        addRequirements(algaeSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // starts moving the arm down
        m_algaeSub.armForward();
    }

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_algaeSub.getAlgaeArmEncoder() < AlgaeConstants.k_armDownGravityPosition) {
            // once the arm has hit down
            m_algaeSub.stopArmMotor();
        }
        m_algaeSub.intake();
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_algaeSub.stopIntakeMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if the arm has already gone down and the limit switch is currently pressed
        return false;
    }
}
