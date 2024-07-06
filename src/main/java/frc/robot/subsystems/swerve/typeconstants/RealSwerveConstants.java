package frc.robot.subsystems.swerve.typeconstants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;

public class RealSwerveConstants implements ISwerveConstants{

    private final double MAX_SPEED_METERS_PER_SECOND = 5.052; //todo: calibrate

    private final Rotation2d MAX_ROTATIONAL_SPEED_PER_SECOND = Rotation2d.fromRadians(10); //todo: calibrate

    private final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(6,0,0); //todo: calibrate

    private final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(6,0,0); //todo: calibrate


    @Override
    public double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public Rotation2d getMaxRotationSpeedPerSecond() {
        return MAX_ROTATIONAL_SPEED_PER_SECOND;
    }

    @Override
    public PIDConstants getTranslationPIDConstants() {
        return TRANSLATION_PID_CONSTANTS;
    }

    @Override
    public PIDConstants getRotationPIDConstants() {
        return ROTATION_PID_CONSTANTS;
    }

}
