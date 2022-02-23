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
// import frc.robot.commands.AutoCommands.OneBallLeftToHuman;
// import frc.robot.commands.AutoCommands.OneBallLeftUp;
// import frc.robot.commands.AutoCommands.ShootForTimed;
import frc.robot.commands.AutoCommands.*;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase {

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;

    private final TrajectoryConfig configForward;
    private final TrajectoryConfig configBackwards;

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory oneBallLeftUp, oneBallLeftToHuman, twoBallLeftUpShoot, twoBallRightUpShoot, oneBallRightToHuman;
    private Trajectory fourBallRight_1, fourBallRight_2;

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
        // autoRoutines.put("1BallLeftUp", createCommandFromTrajectory(oneBallLeftUp));
        // autoRoutines.put("1BallLeftToHuman", createCommandFromTrajectory(oneBallLeftToHuman));
        autoRoutines.put("twoBallLeftUpShoot", createCommandFromTrajectory(twoBallLeftUpShoot));
        autoRoutines.put("twoBallRightUpShoot", createCommandFromTrajectory(twoBallRightUpShoot));
        autoRoutines.put("oneBallRightToHuman", createCommandFromTrajectory(oneBallRightToHuman));

        autoRoutines.put("CMD Group: 1 Ball Left Up", new OneBallLeftUp(getOneBallLeftUp()));
        autoRoutines.put("CMD Group: 1 Ball Left To Human", new OneBallLeftToHuman(getOneBallLeftToHuman()));
        autoRoutines.put("CMD Group: 1 Ball Right To Human", new OneBallRightToHuman(getOneBallRightToHuman()));
        autoRoutines.put("CMD Group: 2 Ball Left Up Shoot", new TwoBallLeftUpShoot(getTwoBallLeftUpShoot()));
        autoRoutines.put("CMD Group: 2 Ball Right Up Shoot", new TwoBallRightUpShoot(getTwoBallRightUpShoot()));
        // autoRoutines.put("CMD Group: 4 Ball Path", new FourBallPathRight(getFourBallPart1(), getFourBallPart2()));
    }

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

    private void defineAutoPaths(){
        oneBallLeftUp = getTransformedTrajectory(PathPlanner.loadPath("1BallLeftUp", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)); 
        oneBallLeftToHuman = getTransformedTrajectory(PathPlanner.loadPath("1BallLeftToHuman", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)); 
        twoBallLeftUpShoot = getTransformedTrajectory(PathPlanner.loadPath("2BallLeftUpShoot", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
        twoBallRightUpShoot = getTransformedTrajectory(PathPlanner.loadPath("2BallRightUpShoot", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
        oneBallRightToHuman = getTransformedTrajectory(PathPlanner.loadPath("1BallRightToHuman", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
        fourBallRight_1 = getTransformedTrajectory(PathPlanner.loadPath("4BallRight_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
        fourBallRight_2 = getTransformedTrajectory(PathPlanner.loadPath("4BallRight_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
    }
 
    public SplitFFRamseteCommand createCommandFromTrajectory(Trajectory trajectory){
        var ramseteController = new RamseteController();
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
                  m_drivetrain.tankDriveVolts(rightVolts, leftVolts);
                  leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                  leftReference.setNumber(rightController.getSetpoint());

                  rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
                  rightReference.setNumber(leftController.getSetpoint());
              },
              m_drivetrain);
        return autoCommand;
      }
    
    public Trajectory getTransformedTrajectory(Trajectory t){
        Pose2d newOrigin = t.getInitialPose();
        Trajectory transformed = t.relativeTo(newOrigin);
        return transformed;
    }

    public SplitFFRamseteCommand getOneBallLeftUp(){
        return createCommandFromTrajectory(oneBallLeftUp);
    }

    public SplitFFRamseteCommand getOneBallLeftToHuman(){
        return createCommandFromTrajectory(oneBallLeftToHuman);
    }

    public SplitFFRamseteCommand getTwoBallLeftUpShoot(){
        return createCommandFromTrajectory(twoBallLeftUpShoot);
    }

    public SplitFFRamseteCommand getTwoBallRightUpShoot(){
        return createCommandFromTrajectory(twoBallRightUpShoot);
    }

    public SplitFFRamseteCommand getOneBallRightToHuman(){
        return createCommandFromTrajectory(oneBallRightToHuman);
    }

    public SplitFFRamseteCommand getFourBallPart1(){
        return createCommandFromTrajectory(fourBallRight_1);
    }

    
    public SplitFFRamseteCommand getFourBallPart2(){
        return createCommandFromTrajectory(fourBallRight_2);
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
