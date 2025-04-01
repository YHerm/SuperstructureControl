package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class DoubleJointedArm extends GBSubsystem{

    private Rotation2d firstJointAngle = new Rotation2d();
    private Rotation2d secondJointAngle = new Rotation2d();

    public DoubleJointedArm(String logPath) {
        super(logPath);
    }

    @Override
    protected void subsystemPeriodic() {
        Logger.recordOutput("Arm/FirstJointAngleDeg", firstJointAngle.getDegrees());
        Logger.recordOutput("Arm/SecondJointAngleDeg", secondJointAngle.getDegrees());
    }

    public Rotation2d getFirstJointAngle() {
        return firstJointAngle;
    }

    public Rotation2d getSecondJointAngle() {
        return secondJointAngle;
    }

    public void setFirstJointAngle(Rotation2d firstJointAngle) {
        this.firstJointAngle = firstJointAngle;
    }

    public void setSecondJointAngle(Rotation2d secondJointAngle) {
        this.secondJointAngle = secondJointAngle;
    }

    public void setAngles(Rotation2d a1, Rotation2d a2){
        setFirstJointAngle(a1);
        setSecondJointAngle(a2);
    }

}
