package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DuckSwerveDrive extends SubsystemBase {
    private final SwerveDriveKinematics m_driveKinematics;
    private final DuckSwerveModule m_FLModule;
    private final DuckSwerveModule m_FRModule;
    private final DuckSwerveModule m_BLModule;
    private final DuckSwerveModule m_BRModule;

    public DuckSwerveDrive(Translation2d driveDimensionsMeters, double turnMinVoltage, double kSLinear, double kVLinear, double kALinear, 
    //convention for the CANID is : front (f) or back (b) : left (l) or right (r) : turn (t) or drive (d) or CANcoder (c)
    int flt, int fld, int flc,
    int frt, int frd, int frc,
    int blt, int bld, int blc,
    int brt, int brd, int brc) {
        //front left, front right, back left, back right
        m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(driveDimensionsMeters.getX()*0.5,driveDimensionsMeters.getY()*0.5),
            new Translation2d(-driveDimensionsMeters.getX()*0.5,driveDimensionsMeters.getY()*0.5)
        );
        //calculate turnMinVoltage by seeing what is the minimum amount of voltage you need to apply to the turn motors to make the robot barely rotate
        //calculate the linear volt gains by having your turn motors be straight and then running a differential drive train test in sysid
        m_FLModule = new DuckSwerveModule(flt, fld, flc, turnMinVoltage, kSLinear, kVLinear, kALinear);
        m_FRModule = new DuckSwerveModule(frt, frd, frc, turnMinVoltage, kSLinear, kVLinear, kALinear);
        m_BLModule = new DuckSwerveModule(blt, bld, blc, turnMinVoltage, kSLinear, kVLinear, kALinear);
        m_BRModule = new DuckSwerveModule(brt, brd, brc, turnMinVoltage, kSLinear, kVLinear, kALinear);
    }

    public void driveRobotCentric(double velocityXMetersPerSecond, double velocityYMetersPerSecond, double angVelocityDegreesPerSecond) {
        SwerveModuleState[] moduleState = m_driveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(velocityXMetersPerSecond, velocityYMetersPerSecond, Units.degreesToRadians(angVelocityDegreesPerSecond))
            );

        //corresponds to displacement configuration
        m_FLModule.updateModuleState(moduleState[0]);
        m_FRModule.updateModuleState(moduleState[1]);
        m_BLModule.updateModuleState(moduleState[2]);
        m_BRModule.updateModuleState(moduleState[3]);
    }

    @Override
    public void periodic() {
        m_FLModule.applyControlAlgorithms();
        m_FRModule.applyControlAlgorithms();
        m_BLModule.applyControlAlgorithms();
        m_BRModule.applyControlAlgorithms();
    }
}
