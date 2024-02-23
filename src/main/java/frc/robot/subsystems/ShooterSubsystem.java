package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shooteTopMotor;
    private CANSparkMax shooteBottomMotor;
    private RelativeEncoder m_encoderTop;
    private RelativeEncoder m_encoderBottom;

    public double maxRPM;
    private LinearFilter topFilter;
    private LinearFilter bottomFilter;
    private LinearFilter averageFilter;
    private double movingAverageVelocity;

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    private GenericEntry topEntry = shooterTab.add("Top Speed", "Nothing").getEntry();
    private GenericEntry bottomEntry = shooterTab.add("Bottom Speed", "Nothing").getEntry();
    private GenericEntry averageEntry = shooterTab.add("Both", "Nothing").getEntry();

    public ShooterSubsystem() {

        this.shooteTopMotor = new CANSparkMax(MotorIds.kTopShooterMotor, MotorType.kBrushless);
        this.shooteBottomMotor = new CANSparkMax(MotorIds.kBottomShooterMotor, MotorType.kBrushless);

        shooteTopMotor.restoreFactoryDefaults();
        shooteBottomMotor.restoreFactoryDefaults();

        m_encoderTop = shooteTopMotor.getEncoder();

        shooteTopMotor.setInverted(true);

        m_encoderBottom = shooteBottomMotor.getEncoder();

        maxRPM = 5700;

        topFilter = LinearFilter.movingAverage(5);
        bottomFilter = LinearFilter.movingAverage(5);
        averageFilter = LinearFilter.movingAverage(5);
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
