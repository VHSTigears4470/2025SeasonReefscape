package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class IntakeCoral extends Command {
    private final CoralSubsystem m_coralSub;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public IntakeCoral(CoralSubsystem coralSub) {
        m_coralSub = coralSub;
        addRequirements(m_coralSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Shoots the coral fast
        m_coralSub.intake();
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //Stops the coral subsystem
        m_coralSub.stopIntakeMotor();
    }

    // Returns true when the command should end. (never happens)
    @Override
    public boolean isFinished() {
        return false;
    }
}