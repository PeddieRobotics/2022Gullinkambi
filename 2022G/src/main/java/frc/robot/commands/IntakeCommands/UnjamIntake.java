package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class UnjamIntake extends CommandBase {
    private Intake m_intake;
    private Hopper m_hopper;

    public UnjamIntake() {
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        addRequirements(m_intake, m_hopper);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSolenoid(true);
    }

    @Override
    public void execute() {
        m_intake.reverseIntake(Constants.INTAKE_SPEED);
        m_hopper.reverseHopper(Constants.HOPPER_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
        m_intake.setIntakeSpeed(Constants.INTAKE_SPEED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
