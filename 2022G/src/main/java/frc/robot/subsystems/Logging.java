package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class Logging extends SubsystemBase{

    private int cycleCount;
    private double[] inputs;

    private static Logging logging;
    private IntegerLogEntry LogCycles; //just used as an example
    private StringLogEntry commandLog;
    private DoubleArrayLogEntry inputLog;

    public Logging() {
        DataLogManager.start("","",10);
        DataLogManager.logNetworkTables(false); //DO NOT REMOVE EVER
        DataLog log = DataLogManager.getLog();

        LogCycles = new IntegerLogEntry(log, "/test/Cycles");
        commandLog = new StringLogEntry(log, "/test/Commands");
        inputLog = new DoubleArrayLogEntry(log, "/test/Inputs");

        inputs= new double[2];
    }

    public static Logging getInstance(){
        if(logging == null){
            logging = new Logging();
        }
        return logging;
      }

    public void startlogging(){
    }

    public void logCommand(String command){
        commandLog.append(command);
    }

    public void logInput(double speed, double turn){
        inputs[0]=speed;
        inputs[1]=turn;
        inputLog.append(inputs);
    }

    public void endLog(){
        commandLog.append("disabled");
    }

    public void teleopInit(){
        cycleCount=0;
    }

    public void teleopPeriodic() {
        cycleCount++;
        LogCycles.append(cycleCount);
    }
}