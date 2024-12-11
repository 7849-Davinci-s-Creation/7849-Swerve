package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class Playsog extends Command {
    private final String file;

    private double startTime;

    Orchestra orchestra = new Orchestra();

    StatusCode status;

    public Playsog(String file) {
        this.file = file;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

        // Add a single device to the orchestra
        orchestra.addInstrument(TunerConstants.DriveTrain.getModule(0).getDriveMotor());
        orchestra.addInstrument(TunerConstants.DriveTrain.getModule(1).getDriveMotor());
        orchestra.addInstrument(TunerConstants.DriveTrain.getModule(3).getDriveMotor());
        orchestra.addInstrument(TunerConstants.DriveTrain.getModule(2).getDriveMotor());

        // Attempt to load the chrp
        status = orchestra.loadMusic(file);
    }

    @Override
    public void execute() {
        if (!status.isOK()) {
            System.out.println("Couldnt load chrp file!");
            return;
        }

        orchestra.play();
    }

    @Override
    public void end(boolean interuppted) {
        orchestra.close();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= 1000 || !status.isOK();
    }

}
