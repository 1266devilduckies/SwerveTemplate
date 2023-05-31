package frc.robot.common;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DuckSwerveModule extends SubsystemBase {
    private final CANSparkMax m_turnMotor;
    private final CANSparkMax m_driveMotor;
    private SwerveModuleState m_moduleState; //represents the velocity state
    
    private final RelativeEncoder m_turnEncoder; //in terms of angular positon on output shaft
    private final AnalogEncoder m_turnEncoderAbsoluteSensor;
    private final RelativeEncoder m_driveEncoder; //in terms of meters per second

    //PF controller (supply static ff as plant)
    private final SparkMaxPIDController m_turnController;
    private final double kTurnStaticVoltage;

    //PF controller
    private final SparkMaxPIDController m_driveController;
    private final SimpleMotorFeedforward m_drivePlant;

    public DuckSwerveModule(int CAN_ID_turnMotor, int CAN_ID_driveMotor, int pwmId, double turnkS, double driveKs, double driveKv, double driveKa) {
        //motor configuration on the burned flash, change via dashboard
        this.m_turnMotor = new CANSparkMax(CAN_ID_turnMotor, MotorType.kBrushless);
        this.m_driveMotor = new CANSparkMax(CAN_ID_driveMotor, MotorType.kBrushless);
        this.m_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        this.m_turnEncoderAbsoluteSensor = new AnalogEncoder(pwmId);
        this.m_turnEncoder = m_turnMotor.getEncoder(); //the motor might need to be inverted if it doesnt rotate CCW+
        this.m_turnController = m_turnMotor.getPIDController();
        this.m_turnController.setFeedbackDevice(m_turnEncoder);

        this.m_driveEncoder = m_driveMotor.getEncoder();
        this.m_driveController = m_driveMotor.getPIDController();
        this.m_driveController.setFeedbackDevice(m_driveEncoder);
        
        this.kTurnStaticVoltage = turnkS;
        this.m_drivePlant = new SimpleMotorFeedforward(driveKs, driveKv, driveKa); //calculate these gains as meters per second
    }

    public void updateModuleState(SwerveModuleState velocityVector) {
        m_moduleState = SwerveModuleState.optimize(velocityVector, Rotation2d.fromDegrees(m_turnEncoder.getPosition()*360.0));
    }

    @Override
    public void periodic() {
        
    }
    public void resetTurnEncoderToAbsoluteEncoder() {
        this.m_turnEncoder.setPosition(this.m_turnEncoderAbsoluteSensor.getAbsolutePosition());
    }

    public void applyControlAlgorithms() {
        if (m_moduleState != null) {
            m_turnController.setReference(m_moduleState.angle.getDegrees()/360.0, ControlType.kPosition, 0, kTurnStaticVoltage);
        
            //closed loop drive
            m_driveController.setReference(m_moduleState.speedMetersPerSecond, ControlType.kVelocity, 0, 
            m_drivePlant.calculate(m_moduleState.speedMetersPerSecond, m_moduleState.speedMetersPerSecond, 0.02)); //rio runs stable at 50hz
        }
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turnEncoder.getPosition()));
    }
}
