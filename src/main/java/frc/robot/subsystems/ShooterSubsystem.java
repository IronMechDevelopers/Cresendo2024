package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
    private SparkPIDController m_pidTopController;
    private RelativeEncoder m_encoderTop;
    private SparkPIDController m_pidBottomController;
    private RelativeEncoder m_encoderBotom;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private LinearFilter filter;
    private double movingAverageVelocity;

    public ShooterSubsystem() {

        this.shooteTopMotor = new CANSparkMax(MotorIds.kTopShooterMotor, MotorType.kBrushless);
        this.shooteBottomMotor = new CANSparkMax(MotorIds.kBottomShooterMotor, MotorType.kBrushless);

        shooteTopMotor.restoreFactoryDefaults();
        shooteBottomMotor.restoreFactoryDefaults();

        m_pidTopController = shooteTopMotor.getPIDController();
        m_encoderTop = shooteTopMotor.getEncoder();

        shooteTopMotor.setInverted(true);

        m_pidBottomController = shooteBottomMotor.getPIDController();
        m_encoderBotom = shooteBottomMotor.getEncoder();

        // PID coefficients
        kP = 6e-4;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        // set PID coefficients
        m_pidTopController.setP(kP);
        m_pidTopController.setI(kI);
        m_pidTopController.setD(kD);
        m_pidTopController.setIZone(kIz);
        m_pidTopController.setFF(kFF);
        m_pidTopController.setOutputRange(kMinOutput, kMaxOutput);

        m_pidBottomController.setP(kP);
        m_pidBottomController.setI(kI);
        m_pidBottomController.setD(kD);
        m_pidBottomController.setIZone(kIz);
        m_pidBottomController.setFF(kFF);
        m_pidBottomController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);

        filter = LinearFilter.movingAverage(5);
        movingAverageVelocity = 0;
    }

    public void stopMotor() {
        shooteTopMotor.set(0);
        shooteBottomMotor.set(0);
    }

    public void setMotorToRPM(double rpm) {
        if (rpm > maxRPM) {
            rpm = maxRPM;
        }
        if (rpm < -1 * maxRPM) {
            rpm = -1 * maxRPM;
        }
        SmartDashboard.putNumber("SetPoint", rpm);
        m_pidTopController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
        m_pidBottomController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public void setMotorToPercent(double speed) {
        if (speed > 1) {
            speed = 1;
        }
        shooteTopMotor.set(speed);
        shooteBottomMotor.set(speed);
    }

    public double getRPM() {
        return movingAverageVelocity;
    }

    @Override
    public void periodic() {

        movingAverageVelocity = filter.calculate(m_encoderTop.getVelocity());
    }

    public Command setMotorToPercentCommand(double percent) {
        return Commands.startEnd(() -> setMotorToPercent(percent), () -> setMotorToPercent(0));
    }

    public Command setMotorToPercentCommand(String strength) {
        double percent = SmartDashboard.getNumber(strength, .20);
        return Commands.startEnd(() -> setMotorToPercent(percent), () -> setMotorToPercent(0));
    }

}
