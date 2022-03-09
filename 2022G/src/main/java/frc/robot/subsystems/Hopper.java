package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.RobotMapGullinkambi;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;
    private LinearFilter bottomSensorFilter, topSensorFilter;

    private CANSparkMax hopperSystem;

    // Define sensors for the hopper to count cargo
    private DigitalInput bottomSensor, topSensor;

    private static UpdateLogs updateLogs = UpdateLogs.getInstance();

    public Hopper() {
        hopperSystem = new CANSparkMax(RobotMapGullinkambi.MOTOR_HOPPER, MotorType.kBrushless);
        hopperSystem.setIdleMode(IdleMode.kBrake);
        hopperSystem.setSmartCurrentLimit(Constants.HOPPER_MAX_CURRENT);
        bottomSensorFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
        topSensorFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

        bottomSensor = new DigitalInput(1);
        topSensor = new DigitalInput(0);
    }

    public void periodic() {
        if(Constants.USE_LOGGING){
            updateLogs.updateHopperLogData();
        }
    }

    public static Hopper getInstance() {
        if (hopper == null) {
            hopper = new Hopper();
        }

        return hopper;
    }

    public void runHopper(double speed){
        hopperSystem.set(-speed); //the speed input needs a multiplier
    }

    public void stopHopper() {
        runHopper(0);
    }

    public void reverseHopper(double speed) {
        runHopper(-speed);
    }

    //Getters
    public double getHopperVelocity(){
        return hopperSystem.get();
    }

    public double getHopperCurrent(){
        return hopperSystem.getOutputCurrent();
    }

    public double getHopperMotorTemperature(){
        return hopperSystem.getMotorTemperature();
    }

    public double getHopperEncoderVelocity(){
        return hopperSystem.getEncoder().getVelocity();
    }

    public double getHopperEncoderPosition(){
        return hopperSystem.getEncoder().getPosition();
    }

    public boolean sensesBallBottom() {
        boolean filteredInput = false;
        if(!bottomSensor.get()){
            double x = bottomSensorFilter.calculate(1);
            SmartDashboard.putNumber("Senses ball bottom", x);
            filteredInput = x > Constants.LOWER_SENSOR_INPUT_THRESHOLD;
        } else {
            double x = bottomSensorFilter.calculate(0);
            SmartDashboard.putNumber("Senses ball bottom", x);
            filteredInput = x > Constants.LOWER_SENSOR_INPUT_THRESHOLD;
        }
        return filteredInput;
    }

    public boolean sensesBallTop() {
        return !topSensor.get();
    }

    public boolean sensesBallTopWithFilter() {
        return updateUpperSensorFilter() > Constants.UPPER_SENSOR_INPUT_THRESHOLD;
    }

    public double updateUpperSensorFilter(){
        double x = 0.0;
        if(sensesBallTop()){
            x = topSensorFilter.calculate(1);
            SmartDashboard.putNumber("Senses ball top", x);
        } else {
            x = topSensorFilter.calculate(0);
            SmartDashboard.putNumber("Senses ball top", x);
        }
        return x;
    }

    public void putSmartDashboardOverrides() {
        SmartDashboard.putNumber("OR: Hopper speed", 0.0);
        SmartDashboard.putNumber("Teleop: Hopper speed", Constants.HOPPER_SPEED);
    }

    public void updateHopperInfoOnDashboard(){
        SmartDashboard.putNumber("Hopper speed", getHopperVelocity());
        SmartDashboard.putBoolean("Lower sensor", sensesBallBottom());
        SmartDashboard.putBoolean("Upper sensor", sensesBallTop());
    }

    public void updateHopperFromDashboard() {
        runHopper(SmartDashboard.getNumber("OR: Hopper speed", 0.0));
    }
}
