package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase {

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;

    private final TrajectoryConfig configForward;
    private final TrajectoryConfig configBackwards;

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory test, sPathTest;

    public Autonomous() {
        autoRoutines = new Hashtable<String,Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        m_drivetrain = Drivetrain.getInstance();

        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                10);

        configForward = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        configBackwards = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint)
                        // Reverse the motors
                        .setReversed(true);

        defineAutoPaths();
        setupAutoRoutines();
        setupAutoSelector();
    }

    public static Autonomous getInstance() {
        if (autonomous == null) {
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public void setupAutoSelector() {
        Enumeration<String> e = autoRoutines.keys();

        while(e.hasMoreElements()){
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        };

        SmartDashboard.putData("Auto Routines", autoRoutineSelector);
    }

    public void setupAutoRoutines() {
        autoRoutines.put("Test Path", createCommandFromTrajectory(test));
        autoRoutines.put("Spath", createCommandFromTrajectory(sPathTest));
    }

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

    private void defineAutoPaths(){
        test = getTransformedTrajectory(PathPlanner.loadPath("StraightPathTest", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
        // test = getTransformedTrajectory(test);
        sPathTest = getTransformedTrajectory(PathPlanner.loadPath("SPathTest", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)); 
    }
 
    public SplitFFRamseteCommand createCommandFromTrajectory(Trajectory trajectory){
        var ramseteController = new RamseteController();
        //ramseteController.setEnabled(false);
        var leftController = new PIDController(Constants.kPDriveVel, 0, 0);
        var rightController = new PIDController(Constants.kPDriveVel, 0, 0);
        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        var leftReference = table.getEntry("left_reference");
        var leftMeasurement = table.getEntry("left_measurement");
        var rightReference = table.getEntry("right_reference");
        var rightMeasurement = table.getEntry("right_measurement");

        SplitFFRamseteCommand autoCommand  =
          new SplitFFRamseteCommand(
              trajectory,
              m_drivetrain::getPose,
              ramseteController,
              new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics,
              m_drivetrain::getWheelSpeeds,
            leftController,
            rightController,
              // RamseteCommand passes volts to the callback
              (leftVolts, rightVolts) -> {
                  m_drivetrain.tankDriveVolts(leftVolts, rightVolts);
                  leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                  leftReference.setNumber(leftController.getSetpoint());

                  rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
                  rightReference.setNumber(rightController.getSetpoint());
              },
              m_drivetrain);
        return autoCommand;
      }
    
    public Trajectory getTransformedTrajectory(Trajectory t){
        Pose2d newOrigin = t.getInitialPose();
        Trajectory transformed = t.relativeTo(newOrigin);
        // Transform2d transform = new Pose2d(0,0, Rotation2d.fromDegrees(0)).minus(t.getInitialPose());
        // Trajectory transformed = t.transformBy(transform);
        return transformed;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
