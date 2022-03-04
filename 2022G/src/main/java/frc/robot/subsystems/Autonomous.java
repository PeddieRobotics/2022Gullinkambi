package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Hashtable;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SplitFFRamseteCommand;
import frc.robot.commands.AutoCommands.FiveBallPathRight;
import frc.robot.commands.AutoCommands.FiveBallPathRightv2;
import frc.robot.commands.AutoCommands.FourBallPathLeft;
import frc.robot.commands.AutoCommands.FourBallPathRight;
import frc.robot.commands.AutoCommands.TwoBallLeftUpShoot;
import frc.robot.commands.AutoCommands.TwoBallRightDownShoot;
import frc.robot.commands.AutoCommands.TwoBallRightUpShoot;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase {

    private final Drivetrain m_drivetrain;
    private static Autonomous autonomous;

    private SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String,Command> autoRoutines;

    private Trajectory sanityCheck;
    private Trajectory oneBallLeftUp, oneBallLeftToHuman, oneBallRightToHuman;
    private Trajectory twoBallLeftUpShoot, twoBallRightUpShoot, twoBallRightDownShoot;
    private Trajectory threeBallRightHuman, threeBallRight_1, threeBallRight_2;
    private Trajectory fourBallRight_1, fourBallRight_2, fourBallLeft_1, fourBallLeft_2, fourBallLeft_3;
    private Trajectory fiveBallRight_1, fiveBallRight_2, fiveBallRight_3, fiveBallRight_4, fiveBallRight_v2_1, fiveBallRight_v2_2, fiveBallRight_v2_3, fiveBallRight_v2_4;

    public Autonomous() {
        autoRoutines = new Hashtable<String,Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        m_drivetrain = Drivetrain.getInstance();

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
        // autoRoutines.put("Sanity Check", new SequentialCommandGroup(new ResetOdometry(sanityCheck.getInitialPose()),createCommandFromTrajectory(sanityCheck)));

        // autoRoutines.put("1BallLeftUp", new SequentialCommandGroup(new ResetOdometry(oneBallLeftUp.getInitialPose()),createCommandFromTrajectory(oneBallLeftUp)));
        // autoRoutines.put("1BallLeftToHuman", new SequentialCommandGroup(new ResetOdometry(oneBallLeftToHuman.getInitialPose()),createCommandFromTrajectory(oneBallLeftToHuman)));
        // autoRoutines.put("twoBallLeftUpShoot", new SequentialCommandGroup(new ResetOdometry(twoBallLeftUpShoot.getInitialPose()),createCommandFromTrajectory(twoBallLeftUpShoot)));
        // autoRoutines.put("twoBallRightUpShoot", new SequentialCommandGroup(new ResetOdometry(twoBallRightUpShoot.getInitialPose()),createCommandFromTrajectory(twoBallRightUpShoot)));
        // autoRoutines.put("oneBallRightToHuman", new SequentialCommandGroup(new ResetOdometry(oneBallRightToHuman.getInitialPose()),createCommandFromTrajectory(oneBallRightToHuman)));

        // autoRoutines.put("3 Ball Part 1 Test:", new SequentialCommandGroup(new ResetOdometry(threeBallRight_1.getInitialPose()),createCommandFromTrajectory(threeBallRight_1)));
        // autoRoutines.put("3 Ball Part 2 Test:", new SequentialCommandGroup(new ResetOdometry(threeBallRight_2.getInitialPose()),createCommandFromTrajectory(threeBallRight_2)));
        
        // autoRoutines.put("4 Ball Part 1 Test:", new SequentialCommandGroup(new ResetOdometry(fourBallRight_1.getInitialPose()),createCommandFromTrajectory(fourBallRight_1)));
        // autoRoutines.put("4 Ball Part 2 Test:", new SequentialCommandGroup(new ResetOdometry(fourBallRight_2.getInitialPose()),createCommandFromTrajectory(fourBallRight_2)));
        
        // autoRoutines.put("CMD Group: 1 Ball Left Up", new OneBallLeftUp(oneBallLeftUp.getInitialPose(), getOneBallLeftUp()));
        // autoRoutines.put("CMD Group: 1 Ball Left To Human", new OneBallLeftToHuman(oneBallLeftToHuman.getInitialPose(), getOneBallLeftToHuman()));
        // autoRoutines.put("CMD Group: 1 Ball Right To Human", new OneBallRightToHuman(oneBallRightToHuman.getInitialPose(), getOneBallRightToHuman()));

        autoRoutines.put("CMD Group: 2 Ball (longer)", new TwoBallLeftUpShoot(twoBallLeftUpShoot.getInitialPose(), getTwoBallLeftUpShoot()));
        autoRoutines.put("CMD Group: 2 Ball (shorter)", new TwoBallRightDownShoot(twoBallRightDownShoot.getInitialPose(), getTwoBallRightDownShoot()));

        // autoRoutines.put("CMD Group: 3 Ball Right To Human", new ThreeBallRightToHuman(threeBallRightHuman.getInitialPose(), getThreeBallRightHuman()));
        // autoRoutines.put("CMD Group: 3 Ball Right", new ThreeBallRight(threeBallRight_1.getInitialPose(), getThreeBallRightPart1(), getThreeBallRightPart2()));

        autoRoutines.put("CMD Group: 4 Ball Path (Right)", new FourBallPathRight(fourBallRight_1.getInitialPose(), getFourBallRightPart1(), getFourBallRightPart2()));

        autoRoutines.put("CMD Group: 4 Ball Path (Left)", new FourBallPathLeft(fourBallLeft_1.getInitialPose(), getFourBallLeftPart1(), getFourBallLeftPart2(), getFourBallLeftPart3()));

        // autoRoutines.put("CMD Group: 5 Ball Path (Right)", new FiveBallPathRight(fiveBallRight_1.getInitialPose(), getFiveBallRightPart1(), getFiveBallRightPart2(), getFiveBallRightPart3(), getFiveBallRightPart4()));

        // autoRoutines.put("CMD Group: 5 Ball Path v2 (Right)", new FiveBallPathRightv2(fiveBallRight_v2_1.getInitialPose(), getFiveBallRightv2Part1(), getFiveBallRightv2Part2(), getFiveBallRightv2Part3(), getFiveBallRightv2Part4()));


    }

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

    private void defineAutoPaths(){
        // sanityCheck = PathPlanner.loadPath("SanityCheck", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        // oneBallLeftUp = PathPlanner.loadPath("1BallLeftUp", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared); 
        // oneBallLeftToHuman = PathPlanner.loadPath("1BallLeftToHuman", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // oneBallRightToHuman = PathPlanner.loadPath("1BallRightToHuman", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);

        twoBallLeftUpShoot = PathPlanner.loadPath("2BallLeftUpShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        twoBallRightUpShoot = PathPlanner.loadPath("2BallRightUpShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        twoBallRightDownShoot = PathPlanner.loadPath("2BallRightDownShoot", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        
        // threeBallRightHuman = PathPlanner.loadPath("3BallRightHuman", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        // threeBallRight_1 = PathPlanner.loadPath("3BallRight_Part1", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        // threeBallRight_2 = PathPlanner.loadPath("3BallRight_Part2", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);

        fourBallRight_1 = PathPlanner.loadPath("4BallRight_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallRight_2 = PathPlanner.loadPath("4BallRight_Part2", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        
        fourBallLeft_1 = PathPlanner.loadPath("4BallLeft_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        fourBallLeft_2 = PathPlanner.loadPath("4BallLeft_Part2", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3);
        fourBallLeft_3 = PathPlanner.loadPath("4BallLeft_Part3", Constants.kMaxSpeedMetersPerSecond*1.3, Constants.kMaxAccelerationMetersPerSecondSquared*1.3, true);

        // fiveBallRight_1 = PathPlanner.loadPath("5BallRight_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // fiveBallRight_2 = PathPlanner.loadPath("5BallRight_Part2", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        // fiveBallRight_3 = PathPlanner.loadPath("5BallRight_Part3", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        // fiveBallRight_4 = PathPlanner.loadPath("5BallRight_Part4", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);

        // fiveBallRight_v2_1 = PathPlanner.loadPath("5BallRight_v2_Part1", Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
        // fiveBallRight_v2_2 = PathPlanner.loadPath("5BallRight_v2_Part2", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        // fiveBallRight_v2_3 = PathPlanner.loadPath("5BallRight_v2_Part3", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);
        // fiveBallRight_v2_4 = PathPlanner.loadPath("5BallRight_v2_Part4", Constants.kMaxSpeedMetersPerSecond*0.5, Constants.kMaxAccelerationMetersPerSecondSquared*0.5);


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

    public SplitFFRamseteCommand getOneBallRightToHuman(){
        return createCommandFromTrajectory(oneBallRightToHuman);
    }

    public SplitFFRamseteCommand getTwoBallLeftUpShoot(){
        return createCommandFromTrajectory(twoBallLeftUpShoot);
    }

    public SplitFFRamseteCommand getTwoBallRightUpShoot(){
        return createCommandFromTrajectory(twoBallRightUpShoot);
    }

    public SplitFFRamseteCommand getTwoBallRightDownShoot(){
        return createCommandFromTrajectory(twoBallRightDownShoot);
    }

    public SplitFFRamseteCommand getThreeBallRightHuman(){
        return createCommandFromTrajectory(threeBallRightHuman);
    }

    public SplitFFRamseteCommand getThreeBallRightPart1(){
        return createCommandFromTrajectory(threeBallRight_1);
    }

    public SplitFFRamseteCommand getThreeBallRightPart2(){
        return createCommandFromTrajectory(threeBallRight_2);
    }

    public SplitFFRamseteCommand getFourBallRightPart1(){
        return createCommandFromTrajectory(fourBallRight_1);
    }
    
    public SplitFFRamseteCommand getFourBallRightPart2(){
        return createCommandFromTrajectory(fourBallRight_2);
    }

    public SplitFFRamseteCommand getFourBallLeftPart1(){
        return createCommandFromTrajectory(fourBallLeft_1);
    }
    
    public SplitFFRamseteCommand getFourBallLeftPart2(){
        return createCommandFromTrajectory(fourBallLeft_2);
    }

    public SplitFFRamseteCommand getFourBallLeftPart3(){
        return createCommandFromTrajectory(fourBallLeft_3);
    }

    public SplitFFRamseteCommand getFiveBallRightPart1(){
        return createCommandFromTrajectory(fiveBallRight_1);
    }
    
    public SplitFFRamseteCommand getFiveBallRightPart2(){
        return createCommandFromTrajectory(fiveBallRight_2);
    }

    public SplitFFRamseteCommand getFiveBallRightPart3(){
        return createCommandFromTrajectory(fiveBallRight_3);
    }

    public SplitFFRamseteCommand getFiveBallRightPart4(){
        return createCommandFromTrajectory(fiveBallRight_4);
    }

    public SplitFFRamseteCommand getFiveBallRightv2Part1(){
        return createCommandFromTrajectory(fiveBallRight_v2_1);
    }
    
    public SplitFFRamseteCommand getFiveBallRightv2Part2(){
        return createCommandFromTrajectory(fiveBallRight_v2_2);
    }

    public SplitFFRamseteCommand getFiveBallRightv2Part3(){
        return createCommandFromTrajectory(fiveBallRight_v2_3);
    }

    public SplitFFRamseteCommand getFiveBallRightv2Part4(){
        return createCommandFromTrajectory(fiveBallRight_v2_4);
    }

}
