package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shooterTopMotor;
    private CANSparkMax shooterBottomMotor;
    public double maxRPM;
    private double movingAverageVelocity;
    private RelativeEncoder m_encoderTop;
    private RelativeEncoder m_encoderBottom;

    public ShooterSubsystem() {

        this.shooterTopMotor = new CANSparkMax(MotorIds.kTopShooterMotor, MotorType.kBrushless);
        this.shooterBottomMotor = new CANSparkMax(MotorIds.kBottomShooterMotor, MotorType.kBrushless);

        shooterTopMotor.restoreFactoryDefaults();
        shooterBottomMotor.restoreFactoryDefaults();

        m_encoderTop = shooterTopMotor.getEncoder();
        m_encoderBottom = shooterBottomMotor.getEncoder();

        shooterTopMotor.setInverted(true);

        maxRPM = 5700;
    }

    public void stopMotor() {
        setMotorToPercent(0);
    }

    public void setMotorToPercent(double speed) {
        if (speed > 1) {
            speed = 1;
        }

        shooterTopMotor.set(speed);
        shooterBottomMotor.set(speed);
        SmartDashboard.putNumber("Shooter Percent", speed);
    }

    public void setMotorToPercent(String strength) {
        double percent = SmartDashboard.getNumber(strength, 0.45);
        setMotorToPercent(percent);
    }

    public double getRPM() {
        return movingAverageVelocity;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Shooter Velocity", m_encoderTop.getVelocity());
        SmartDashboard.putNumber("Bottom Shooter Velocity", m_encoderBottom.getVelocity());

    }

    public Command setMotorToPercentCommand(double percent) {
        return Commands.startEnd(() -> setMotorToPercent(percent), () -> setMotorToPercent(0), this);
    }

    public Command setMotorToPercentCommand(String strength) {
        return Commands.startEnd(() -> setMotorToPercent(strength), () -> setMotorToPercent(0), this);
    }

        public Command setMotorToInvesePercentCommand() {
        return Commands.startEnd(() -> setMotorToPercent(-.25), () -> setMotorToPercent(0), this);
    }

    public Command warmUpMotorToPercentCommand(String strength) {
        return Commands.runOnce(() -> setMotorToPercent(strength), this);
    }

   

}
