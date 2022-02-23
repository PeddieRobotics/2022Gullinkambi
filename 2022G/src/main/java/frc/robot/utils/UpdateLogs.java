package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import frc.robot.utils.LoggingCustoms.LoggerCustom;

public class UpdateLogs {

    private static Drivetrain m_drivetrain;
    private static Climber m_climber;
    private static Flywheel m_flywheel;
    private static Hopper m_hopper;
    private static Intake m_intake;
    private static Limelight m_limelight;

    private static UpdateLogs instance;

    private UpdateLogs(){
        m_drivetrain = Drivetrain.getInstance();
        m_climber = Climber.getInstance();
        m_flywheel = Flywheel.getInstance();
        m_hopper = Hopper.getInstance();
        m_intake = Intake.getInstance();
        m_limelight = Limelight.getInstance();
    }

    public static UpdateLogs getInstance(){
        if (instance == null){
            instance = new UpdateLogs();
        }
        return instance;
    }

    public void updateDrivetrainLogData(){
        double pose[] = new double[3];

        //Encoders
        LoggerCustom.getInstance().recordOutput("Drivetrain/LeftEncPosition", m_drivetrain.getLeftEncoderPosition());
        LoggerCustom.getInstance().recordOutput("Drivetrain/LeftEncVelocity", m_drivetrain.getLeftEncoderVelocity());
        LoggerCustom.getInstance().recordOutput("Drivetrain/RightEncPosition", m_drivetrain.getRightEncoderPosition());
        LoggerCustom.getInstance().recordOutput("Drivetrain/RightEncVelocity", m_drivetrain.getRightEncoderVelocity());   
        
        //Pose
        pose[0] = m_drivetrain.getPose().getTranslation().getX();
        pose[1] = m_drivetrain.getPose().getTranslation().getY();
        pose[2] = Rotation2d.fromDegrees(m_drivetrain.getHeading()).getRadians();
        LoggerCustom.getInstance().recordOutput("OdometryArray", pose);

        //Drive Setpoints
        LoggerCustom.getInstance().recordOutput("Drivetrain/SpeedSetpoint", m_drivetrain.getSpeedSetpoint());
        LoggerCustom.getInstance().recordOutput("Drivetrain/TurnSetpoint", m_drivetrain.getTurnSetpoint());

        //Drive Motor Data
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorVelocities/LeftMasterVelocity", m_drivetrain.getLeftMasterVelocity());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorVelocities/LeftFollowerVelocity", m_drivetrain.getLeftFollowerVelocity());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorVelocities/LeftFollower2Velocity", m_drivetrain.getLeftFollower2Velocity());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorVelocities/RightMasterVelocity", m_drivetrain.getRightMasterVelocity());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorVelocities/RightFollowerVelocity", m_drivetrain.getRightFollowerVelocity());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorVelocities/RightFollower2Velocity", m_drivetrain.getRightFollower2Velocity());

        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorCurrents/LeftMasterCurrent", m_drivetrain.getLeftMasterCurrent());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorCurrents/LeftFollowerCurrent", m_drivetrain.getLeftFollowerCurrent());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorCurrents/LeftFollower2Current", m_drivetrain.getLeftFollower2Current());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorCurrents/RightMasterCurrent", m_drivetrain.getRightMasterCurrent());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorCurrents/RightFollowerCurrent", m_drivetrain.getRightFollowerCurrent());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorCurrents/RightFollower2Current", m_drivetrain.getRightFollower2Current());
        
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorTemperatures/LeftMasterTemperature", m_drivetrain.getLeftMasterMotorTemperature());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorTemperatures/LeftFollowerTemperature", m_drivetrain.getLeftFollowerMotorTemperature());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorTemperatures/LeftFollower2Temperature", m_drivetrain.getLeftFollower2MotorTemperature());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorTemperatures/RightMasterTemperature", m_drivetrain.getRightMasterMotorTemperature());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorTemperatures/RightFollowerTemperature", m_drivetrain.getRightFollowerMotorTemperature());
        LoggerCustom.getInstance().recordOutput("Drivetrain/MotorTemperatures/RightFollower2Temperature", m_drivetrain.getRightFollower2MotorTemperature());

        LoggerCustom.getInstance().recordOutput("Drivetrain/HeadingValue", m_drivetrain.getHeading());
    }

    public void updateClimberLogData(){
        LoggerCustom.getInstance().recordOutput("Climber/ClimberArmSensorValue", m_climber.sensesArm());

        //Encoder Data
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor1EncoderVelocity", m_climber.getarmMotor1EncoderVelocity());
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor1EncoderPosition", m_climber.getarmMotor1EncoderPosition());
        
        //Winch Motor Data
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor1Velocity", m_climber.getarmMotor1Velocity());
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor2Velocity", m_climber.getarmMotor2Velocity());

        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor1Current", m_climber.getarmMotor1Current());
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor2Current", m_climber.getarmMotor2Current());
        
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor1Temperature", m_climber.getarmMotor1Temperature());
        LoggerCustom.getInstance().recordOutput("Climber/WinchMotor2Temperature", m_climber.getarmMotor2Temperature());
    }

    public void updateFlywheelLogData(){
        //Encoders
        LoggerCustom.getInstance().recordOutput("Flywheel/FlywheelEncPosition", m_flywheel.getFlywheelEncoderPosition());
        LoggerCustom.getInstance().recordOutput("Flywheel/FlywheelEncVelocity", m_flywheel.getFlywheelEncoderVelocity());

        //Flywheel Setpoints
        LoggerCustom.getInstance().recordOutput("Flywheel/RPMSetpoint", m_flywheel.getFlywheelSetpoint());

        //Motor Data
        LoggerCustom.getInstance().recordOutput("Flywheel/PrimaryFlywheelVelocity", m_flywheel.getPrimaryFlywheelVelocity());
        LoggerCustom.getInstance().recordOutput("Flywheel/SecondaryFlywheelVelocity", m_flywheel.getSecondaryFlywheelVelocity());
        
        LoggerCustom.getInstance().recordOutput("Flywheel/PrimaryFlywheelCurrent", m_flywheel.getPrimaryFlywheelCurrent());
        LoggerCustom.getInstance().recordOutput("Flywheel/SecondaryFlywheel2Current", m_flywheel.getSecondaryFlywheelCurrent());
        
        LoggerCustom.getInstance().recordOutput("Flywheel/PrimaryFlywheelTemperature", m_flywheel.getPrimaryFlywheelMotorTemperature());
        LoggerCustom.getInstance().recordOutput("Flywheel/SecondaryFlywheel2Temperature", m_flywheel.getSecondaryFlywheelMotorTemperature());
        
        LoggerCustom.getInstance().recordOutput("Flywheel/HoodUp", m_flywheel.isHoodUp());
        LoggerCustom.getInstance().recordOutput("Flywheel/ShooterLock", m_flywheel.getShooterLock());
    }

    public void updateHopperLogData(){
        LoggerCustom.getInstance().recordOutput("Hopper/BottomSensorValue", m_hopper.sensesBallBottom());
        LoggerCustom.getInstance().recordOutput("Hopper/TopSensorValue", m_hopper.sensesBallTop());

        //EncoderData
        LoggerCustom.getInstance().recordOutput("Hopper/HopperMotorEncoderVelocity", m_hopper.getHopperEncoderVelocity());
        LoggerCustom.getInstance().recordOutput("Hopper/HopperMotorEncoderCurrent", m_hopper.getHopperEncoderPosition());

        //Hopper Motor Data
        LoggerCustom.getInstance().recordOutput("Hopper/HopperMotorVelocity", m_hopper.getHopperVelocity());
        LoggerCustom.getInstance().recordOutput("Hopper/HopperMotorCurrent", m_hopper.getHopperCurrent());
        LoggerCustom.getInstance().recordOutput("Hopper/HopperMotorTemperature", m_hopper.getHopperMotorTemperature());
    }

    public void updateIntakeLogData(){
        LoggerCustom.getInstance().recordOutput("Intake/IntakeSolenoidOn", m_intake.getSolenoidState());

        //Intake Motor Data
        LoggerCustom.getInstance().recordOutput("Intake/IntakeMotorCurrent", m_intake.getIntakeCurrent());
        LoggerCustom.getInstance().recordOutput("Intake/IntakeMotorTemperature", m_intake.getIntakeMotorTemperature());
    }

    public void updateLimelightLogData(){
        LoggerCustom.getInstance().recordOutput("Limelight/TargetHeightPixels", m_limelight.getTvert());
        LoggerCustom.getInstance().recordOutput("Limelight/TargetWidthPixels", m_limelight.getThor());
        LoggerCustom.getInstance().recordOutput("Limelight/HorizonalOffsetDegrees", m_limelight.getTx());
        LoggerCustom.getInstance().recordOutput("Limelight/VerticalOffsetDegrees", m_limelight.getTy());
        LoggerCustom.getInstance().recordOutput("Limelight/TargetArea%ofImmues", m_limelight.getTa());
    }
}
