package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shooteTopMotor;
    private CANSparkMax shooteBottomMotor;
    public double maxRPM;
    private double movingAverageVelocity;

    public ShooterSubsystem() {

        this.shooteTopMotor = new CANSparkMax(MotorIds.kTopShooterMotor, MotorType.kBrushless);
        this.shooteBottomMotor = new CANSparkMax(MotorIds.kBottomShooterMotor, MotorType.kBrushless);

        shooteTopMotor.restoreFactoryDefaults();
        shooteBottomMotor.restoreFactoryDefaults();

        shooteTopMotor.getEncoder();

        shooteTopMotor.setInverted(true);

        shooteBottomMotor.getEncoder();

        maxRPM = 5700;

        LinearFilter.movingAverage(5);
        LinearFilter.movingAverage(5);
        LinearFilter.movingAverage(5);
        movingAverageVelocity = 0;
    }

    public void stopMotor() {
        shooteTopMotor.set(0);
        shooteBottomMotor.set(0);
    }

    public void setMotorToPercent(double speed) {
        if (speed > 1) {
            speed = 1;
        }

        shooteTopMotor.set(speed);
        shooteBottomMotor.set(speed);
    }

    public void setMotorToPercent(String strength) {
        double percent = SmartDashboard.getNumber(strength, .45);
        setMotorToPercent(percent);
    }

    public double getRPM() {
        return movingAverageVelocity;
    }

    @Override
    public void periodic() {

        // double a = topFilter.calculate(m_encoderTop.getVelocity());
        // double b = bottomFilter.calculate(m_encoderBottom.getVelocity());
        // double c = averageFilter.calculate((m_encoderTop.getVelocity() +
        // m_encoderBottom.getVelocity()) / 2.0);

        // topEntry.setDouble(a);
        // bottomEntry.setDouble(b);
        // averageEntry.setDouble(c);

    }

    public Command setMotorToPercentCommand(double percent) {

        return Commands.startEnd(() -> setMotorToPercent(percent), () -> setMotorToPercent(0));
    }

    public Command setMotorToPercentCommand(String strength) {
        return Commands.startEnd(() -> setMotorToPercent(strength), () -> setMotorToPercent(0));
    }

}
