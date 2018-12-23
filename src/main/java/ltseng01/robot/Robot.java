package ltseng01.robot;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Robot extends IterativeRobot {

    private static WPI_TalonSRX leftFront;
    private static WPI_TalonSRX leftBack;
    private static WPI_TalonSRX rightFront;
    private static WPI_TalonSRX rightBack;

    private static SpeedControllerGroup leftDrive;
    private static SpeedControllerGroup rightDrive;

    private static DifferentialDrive differentialDrive;

//    private static Encoder leftEncoder;
//    private static Encoder rightEncoder;

    private static XboxController xboxController;

    private static ArrayList<double[]> leftProfile;     // pos., vel., accel.
    private static ArrayList<double[]> rightProfile;

    private static MotionProfileStatus leftMPStatus;
    private static MotionProfileStatus rightMPStatus;

    @Override
    public void robotInit() {

        xboxController = new XboxController(0);

        leftFront = new WPI_TalonSRX(1);
        leftBack = new WPI_TalonSRX(2);
        rightFront = new WPI_TalonSRX(3);
        rightBack = new WPI_TalonSRX(4);

        rightFront.configPeakOutputForward(1, 0);
        rightFront.configPeakOutputReverse(-1, 0);

        leftDrive = new SpeedControllerGroup(leftFront, leftBack);
        rightDrive = new SpeedControllerGroup(rightFront, rightBack);

        differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
        differentialDrive.setSafetyEnabled(false);

        leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

        rightFront.setInverted(true);
        rightBack.setInverted(true);

        rightFront.setSensorPhase(false);

        rightDrive.setInverted(true);

//        leftEncoder = new Encoder(0, 1, true);
//        rightEncoder = new Encoder(3, 4);

        resetEncoders();

        leftMPStatus = new MotionProfileStatus();
        rightMPStatus = new MotionProfileStatus();

    }

    @Override
    public void robotPeriodic() {
//        SmartDashboard.putNumber("Left Encoder Accum. Ticks", leftEncoder.get());
//        SmartDashboard.putNumber("Right Encoder Accum. Ticks", rightEncoder.get());
//        SmartDashboard.putNumber("Left Encoder Velocity", leftEncoder.getRate());
//        SmartDashboard.putNumber("Right Encoder Velocity", rightEncoder.getRate());

        int leftFrontPosition = leftFront.getSelectedSensorPosition(0);
        int rightFrontPosition = rightFront.getSelectedSensorPosition(0);
        int leftFrontVelocity = leftFront.getSelectedSensorVelocity(0);
        int rightFrontVelocity = rightFront.getSelectedSensorVelocity(0);

        SmartDashboard.putNumber("Left Encoder Accum. Ticks", leftFrontPosition);
        SmartDashboard.putNumber("Right Encoder Accum. Ticks", rightFrontPosition);
        SmartDashboard.putNumber("Left Encoder Velocity", leftFrontVelocity);
        SmartDashboard.putNumber("Right Encoder Velocity", rightFrontVelocity);

        SmartDashboard.putNumber("Left Encoder Distance, m", ticksToMeters(leftFrontPosition));
        SmartDashboard.putNumber("Right Encoder Distance, m", ticksToMeters(rightFrontPosition));
        SmartDashboard.putNumber("Left Encoder Velocity, mps", ticksToMeters(leftFrontVelocity * 10));  // vel * 10 to go from 100ms -> 1s
        SmartDashboard.putNumber("Right Encoder Velocity, mps", ticksToMeters(rightFrontVelocity * 10));

        /*

        2048 resolution = 8192 ticks

        8192 ticks = 1 rotation
        1 rotation = 2 * pi * .1016m

        (position / 8192) * (Math.PI * 0.1016)

        -----

        8192 ticks per 100milliseconds * 10 = ticks per second

        (velocity / 10 / 8192) * (Math.PI * 0.1016)

         */

    }

    @Override
    public void autonomousInit() {

        System.out.println("-- AUTONOMOUS INIT --");

        leftBack.set(ControlMode.Follower, 1);      // Left Front
        rightBack.set(ControlMode.Follower, 3);     // Right Front

        leftFront.getMotionProfileStatus(leftMPStatus);
        rightFront.getMotionProfileStatus(rightMPStatus);

        leftFront.config_kF(0, 0.134, 20);
        rightFront.config_kF(0, 0.134, 20);

        leftProfile = readCSVMotionProfileFile("/home/lvuser/paths/path_left.csv");
        rightProfile = readCSVMotionProfileFile("/home/lvuser/paths/path_right.csv");

        System.out.println("Left Profile Count: " + leftProfile.size());
        System.out.println("Right Profile Count: " + rightProfile.size());

//        System.out.println("Left Profile");
//        for (double[] segment : leftProfile) {
//            for (double point : segment) {
//                System.out.println(point);
//            }
//        }
//
//        System.out.println("Right Profile");
//        for (double[] segment : rightProfile) {
//            for (double point : segment) {
//                System.out.println(point);
//            }
//        }

        if (leftMPStatus.hasUnderrun)
            leftFront.clearMotionProfileHasUnderrun(0);

        if (rightMPStatus.hasUnderrun)
            rightFront.clearMotionProfileHasUnderrun(0);

        leftFront.clearMotionProfileTrajectories();
        leftFront.configMotionProfileTrajectoryPeriod(0, 10);
        rightFront.clearMotionProfileTrajectories();
        rightFront.configMotionProfileTrajectoryPeriod(0, 10);

        loadTrajectoryToTalon(leftFront, leftProfile);
        loadTrajectoryToTalon(rightFront, rightProfile);

        System.out.println("-- AUTONOMOUS INIT END --");
    }

    private void loadTrajectoryToTalon(TalonSRX talonSRX, ArrayList<double[]> profile) {
        TrajectoryPoint point = new TrajectoryPoint();

        for (int i = 0; i < profile.size(); i++) {
            point.position = metersToTicks(profile.get(i)[0]);     // meters -> rotations -> ticks
            point.velocity = metersToTicks(profile.get(i)[1]) / 10.0;     // meters/second -> ticks/sec -> ticks/100ms
            point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_50ms;
            point.profileSlotSelect0 = 0;

            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == profile.size();

            talonSRX.pushMotionProfileTrajectory(point);

        }

        System.out.println("Loaded Trajectory");

    }

    private int metersToTicks(double meters) {
        return (int) ((meters / (Math.PI * 0.1016)) * (8192.0));
    }

    private double ticksToMeters(int ticks) {
        return (ticks / 8192.0) * (Math.PI * 0.1016);       // assuming 8192 ticks per rotation, 0.1016 = diameter in meters
    }

    @Override
    public void autonomousPeriodic() {

        leftFront.getMotionProfileStatus(leftMPStatus);
        rightFront.getMotionProfileStatus(rightMPStatus);

        leftFront.processMotionProfileBuffer();
        rightFront.processMotionProfileBuffer();

        if (leftMPStatus.btmBufferCnt > 50 || leftMPStatus.topBufferCnt == 0)
            leftFront.set(ControlMode.MotionProfile, 1);

        if (rightMPStatus.btmBufferCnt > 50 || rightMPStatus.topBufferCnt == 0)
            rightFront.set(ControlMode.MotionProfile, 1);

        if (leftMPStatus.isLast) {
            System.out.println("Done with Left Profile");
            leftFront.set(ControlMode.MotionProfile, 0);
        }

        if (rightMPStatus.isLast) {
            System.out.println("Done with Right Profile");
            rightFront.set(ControlMode.MotionProfile, 0);
        }

    }

    @Override
    public void teleopInit() {
        leftFront.set(ControlMode.PercentOutput, 0.0);
        leftBack.set(ControlMode.PercentOutput, 0.0);
        rightFront.set(ControlMode.PercentOutput, 0.0);
        rightBack.set(ControlMode.PercentOutput, 0.0);

    }

    @Override
    public void teleopPeriodic() {

        // Arcade Drive
        differentialDrive.arcadeDrive(-xboxController.getY(GenericHID.Hand.kLeft),
                0.5 * xboxController.getX(GenericHID.Hand.kRight));

        // Tank Drive
//        differentialDrive.tankDrive(0.5 * -xboxController.getY(GenericHID.Hand.kLeft),
//                0.5 * -xboxController.getY(GenericHID.Hand.kRight));

        if (xboxController.getXButtonPressed()) {
            resetEncoders();
        }

    }

    @Override
    public void testPeriodic() {

    }

    private static void resetEncoders() {
//        leftEncoder.reset();
//        rightEncoder.reset();
        leftFront.setSelectedSensorPosition(0, 0, 10);
        rightFront.setSelectedSensorPosition(0, 0, 10);
    }

    private static ArrayList<double[]> readCSVMotionProfileFile(String path) {

        ArrayList<double[]> pathSegments = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {

            String line;
            String csvDelimiter = ",";

            while ((line = br.readLine()) != null) {
                String[] segment = line.split(csvDelimiter);

                double[] convertedSegment = Arrays.stream(segment)
                        .mapToDouble(Double::parseDouble)
                        .toArray();

                pathSegments.add(convertedSegment);
            }

        } catch (IOException ex) {
            DriverStation.reportError("Unable to read motion profile file!", true);
        }

        return pathSegments;

    }

}
