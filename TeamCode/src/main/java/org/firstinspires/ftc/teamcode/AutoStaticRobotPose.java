package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class AutoStaticRobotPose extends LinearOpMode {

    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 11.25;
    public static final double CENTER_WHEEL_OFFSET = -1.125;
    public static final double WHEEL_DIAMETER = 1.88976;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "intake");
        rightEncoder = new MotorEx(hardwareMap, "climber");
        perpEncoder = new MotorEx(hardwareMap, "rightLift");

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        waitForStart();

        while (!isStopRequested()) {
            // run autonomous

            // update positions
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();
        }
    }

}
