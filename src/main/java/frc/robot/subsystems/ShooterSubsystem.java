package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        SmartDashboard.putNumber("Shoot Speed", .50);

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
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

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
        // speed = SmartDashboard.getNumber("Shoot Speed", .90);
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
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            m_pidTopController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            m_pidTopController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidTopController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pidTopController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidTopController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidTopController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        movingAverageVelocity = filter.calculate(m_encoderTop.getVelocity());
        SmartDashboard.putNumber("Shooter Speed", movingAverageVelocity);
        SmartDashboard.putNumber("Shooter Speed Direct", m_encoderTop.getVelocity());
    }

}
