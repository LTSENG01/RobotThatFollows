package ltseng01.robot;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

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

        leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        rightFront.setSensorPhase(true);

//        leftEncoder = new Encoder(0, 1, true);
//        rightEncoder = new Encoder(3, 4);

        resetEncoders();

    }

    @Override
    public void robotPeriodic() {
//        SmartDashboard.putNumber("Left Encoder Accum. Ticks", leftEncoder.get());
//        SmartDashboard.putNumber("Right Encoder Accum. Ticks", rightEncoder.get());
//        SmartDashboard.putNumber("Left Encoder Velocity", leftEncoder.getRate());
//        SmartDashboard.putNumber("Right Encoder Velocity", rightEncoder.getRate());

        double leftFrontPosition = leftFront.getSelectedSensorPosition(0);
        double rightFrontPosition = rightFront.getSelectedSensorPosition(0);
        double leftFrontVelocity = leftFront.getSelectedSensorVelocity(0);
        double rightFrontVelocity = rightFront.getSelectedSensorVelocity(0);

        SmartDashboard.putNumber("Left Encoder Accum. Ticks", leftFrontPosition);
        SmartDashboard.putNumber("Right Encoder Accum. Ticks", rightFrontPosition);
        SmartDashboard.putNumber("Left Encoder Velocity", leftFrontVelocity);
        SmartDashboard.putNumber("Right Encoder Velocity", rightFrontVelocity);

        SmartDashboard.putNumber("Left Encoder Distance, m", (leftFrontPosition / 8192) * (Math.PI * 0.1016));
        SmartDashboard.putNumber("Right Encoder Distance, m", (rightFrontPosition / 8192) * (Math.PI * 0.1016));
        SmartDashboard.putNumber("Left Encoder Velocity, mps", ((leftFrontVelocity * 10) / 8192) * (Math.PI * 0.1016));
        SmartDashboard.putNumber("Right Encoder Velocity, mps", ((rightFrontVelocity * 10) / 8192) * (Math.PI * 0.1016));

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
        leftProfile = readCSVMotionProfileFile("/home/lvuser/paths/path_left.csv");
        rightProfile = readCSVMotionProfileFile("/home/lvuser/paths/path_right.csv");

        System.out.println("Left Profile");
        for (double[] segment : leftProfile) {
            for (double point : segment) {
                System.out.println(point);
            }
        }

        System.out.println("Right Profile");
        for (double[] segment : rightProfile) {
            for (double point : segment) {
                System.out.println(point);
            }
        }

        for (int i = 0; i < leftProfile.size(); i++) {

        }

        leftFront.config_kF(0, 0.027, 20);
        rightFront.config_kF(0, 0.027, 20);


    }

    @Override
    public void autonomousPeriodic() {

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
                0.0 * xboxController.getX(GenericHID.Hand.kRight));

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
