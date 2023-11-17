package org.firstinspires.ftc.teamcode.drive.opmode;// import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.view.View;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//import org.firstinspires.ftc.teamcode.SolomonRandom.MB1242Ex;

import java.util.List;

@Autonomous(name="FibbyAuto24", group="Robot")
//@Disabled
public class FibbyAuto24 extends LinearOpMode {
    //----------------------------------------------------------------------------------------------------
    // DECLARING MOTOR VARIABLES
    private DcMotorEx leftFront  = null;
    private DcMotorEx leftRear   = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear  = null;

    private double leftFrontPower  = 0;
    private double leftRearPower   = 0;
    private double rightFrontPower = 0;
    private double rightRearPower  = 0;

    // Sensor Variables
    private double leftFrontEncoder;
    private double leftRearEncoder;
    private double rightFrontEncoder;
    private double rightRearEncoder;
    private double leftFrontVelocity;
    private double leftRearVelocity;
    private double rightFrontVelocity;
    private double rightRearVelocity;
    private double leftLiftEncoder;
    private double rightLiftEncoder;
    private double leftLiftAmps;
    private double rightLiftAmps;
    private double intake_reading;
    private double climber_reading;
    private boolean lowerLimitSwitch;

    private DcMotorEx leftLift    = null;  // these are the arm motors
    private DcMotorEx rightLift = null;
    private DcMotor intake = null;
    private DcMotor climber = null;

    private double liftPower = 0;


    //----------------------------------------------------------------------------------------------------
    // DECLARING SERVO VARIABLES

    public Servo kickL;
    public Servo kickR;
    public CRServo Intake_Roller;
    private CRServo Deposit;
    double kickPosition = 0.04;

    //----------------------------------------------------------------------------------------------------
    // DECLARING SENSOR VARIABLES
    private IMU imu = null;

    //private DistanceSensor distIntake;
    //private DistanceSensor distForZero;
    //private NormalizedColorSensor frontColorDist;
    View relativeLayout;

    DigitalChannel lowerLimit;

    //private MB1242Ex rangeSensor;

    //----------------------------------------------------------------------------------------------------
    // CONSTANT VARIABLES
    static final boolean SeeTelemetry = true;
    static final double P_DRIVE_GAIN = 0.03;    // larger is more responsive, but also less stable

    static final double tickToINCH = 345;  // reading of the encoder per inch

    static final int tolerance = 10;

    static final double driveMaxVelocity = 2920; // 12 Volt battery tested at 2680, 13.73 battery was at 3120, and 13 volts was 2920

    //----------------------------------------------------------------------------------------------------
    // REUSABLE VARIABLES
    int TSEplacement = 0;  // for camera detection
    int objects = 0;

    double corrHeading;
    double heading = 0;
    double diffCorrection;
    double delta;


    boolean questionAnswered = false;
    boolean redSide = false;
    boolean rightSide = false;

    boolean pixelDrop = false;

    double desiredCourse = 0;   // telemetry use

    //----------------------------------------------------------------------------------------------------
    // CAMERA/VUFORIA VARIABLES

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "RedAndBlue.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    private static final String[] LABELS = {
            "blue 1",
            "blue 2",
            "red 1",
            "red 2"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE MOTORS (hardware mapping)
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        climber = hardwareMap.get(DcMotorEx.class, "climber");
        Intake_Roller = hardwareMap.get(CRServo.class,"intakeRoller");

        Deposit = hardwareMap.get(CRServo.class,"deposit");

        kickL = hardwareMap.get(Servo.class,"kickL");
        kickR = hardwareMap.get(Servo.class,"kickR");


        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SET THE MOTOR DIRECTION (to make it drive correctly)
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD); // changed 2-23-23 from forward to fix encoder issue
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        //parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        //perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        climber.setDirection(DcMotorEx.Direction.REVERSE);


        // RESET THE ENCODERS AND SET THE MOTORS TO BRAKE MODE
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // ALLOW OR NOT ALLOW ENCODERS
        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // SET POWER
        leftLift.setPower(0);
        rightLift.setPower(0);

        intake.setPower(0);
        climber.setPower(0);

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SENSORS (hardware mapping)
        imu = hardwareMap.get(IMU.class, "imu");

        //distIntake = hardwareMap.get(DistanceSensor.class, "dist_intake2");
        //distForZero = hardwareMap.get(DistanceSensor.class, "dist_for_zero");
        //frontColorDist = hardwareMap.get(NormalizedColorSensor.class, "dist_intake");

        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");

        lowerLimit.setMode(DigitalChannel.Mode.INPUT);

        //rangeSensor = hardwareMap.get(MB1242Ex.class, "rangeSensor");

        // DEFINE HUB ORIENTATION
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // INITIALIZE THE IMU WITH THIS MOUNTING ORIENTATION
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // RESET THE HEADING
        resetHeading();
        telemetry.addData(">", "gyro reset good");
        sleep(2000);


        //----------------------------------------------------------------------------------------------------
        //SET LIFT STUFF

        leftLift.setPower(0);
        rightLift.setPower(0);

        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (lowerLimitSwitch) {
            kickL.setPosition(kickPosition);
            kickR.setPosition(1-kickPosition);
        }

        //---------------------------------------------------------------------------------------------------
        //CAMERA STUFF
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        //----------------------------------------------------------------------------------------------------
        // QUESTIONS

        telemetry.addLine("Blue Side - □ | Red Side - O");
        telemetry.update();

        while (questionAnswered == false) {
            if (gamepad1.b || gamepad2.b) {
                redSide = true;
                questionAnswered = true;

            } else if (gamepad1.x || gamepad2.x) {
                redSide = false;
                questionAnswered = true;

            } else {
                questionAnswered = false;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }
        }

        telemetry.addLine("Left Side - dpad left | Right Side - dpad right");
        telemetry.update();

        questionAnswered = false;
        while (questionAnswered == false) {
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                rightSide = true;
                questionAnswered = true;

            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                rightSide = false;
                questionAnswered = true;

            } else {
                questionAnswered = false;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }
        }

        telemetry.addLine("pixelDrop - dpadup");
        telemetry.update();

        questionAnswered = false;
        while (questionAnswered == false){
            if (gamepad1.dpad_up || gamepad2.dpad_up){
                pixelDrop = true;
                questionAnswered = true;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }

        }

        telemetry.addLine("Questions Answered");
        telemetry.update();

        //---------------------------------------------------------------------------------------------------
        //CAMERA STUFF

        while (!isStarted()) {

            telemetryTfod();

            if (objects == 0) {
                TSEplacement = 3;
            }
            telemetry.addData("TSE", TSEplacement);
        }

        //----------------------------------------------------------------------------------------------------
        // ENTER CODE BELOW
        if (pixelDrop){
            resetRuntime();
            pixelDrop();
        }
//
        else {
            // do nothing
        }

        resetRuntime();
    }


    //----------------------------------------------------------------------------------------------------
    // OUR AUTONOMOUS RUNS
    //----------------------------------------------------------------------------------------------------

    public void pixelDrop()
    {
        if (redSide == false){
            if(TSEplacement == 1 && rightSide == false){
                // strafe left 9 IN
                GyroStrafeENC(9, 0.5, "left", 0);
                sleep(150);
                //drive forward 12 IN
                GyroDriveENC(12, 0.5, 0,true, true, true, true, 7);
                //spit out purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                //drive backwards 11 IN
                GyroDriveENC(11, 0.5, 0,false, false, true, true, 3);
                //Turn 90 degrees right
                GyroSpin(0.5, 90,15);
                //raise the lift and extend the kicker
                liftENC(600, 0.75);
                kick(true);
                //drive backwards 12 IN
                GyroDriveENC(-12, 0.5, 90, true, false, true, true, -6);
                //strafe left 12 IN
                GyroStrafeENC(12, 0.5, "left", 90);
                //drive backwards 6 IN
                GyroDriveENC(-6, 0.2, 90, true, false, true, true, -6);
                //drop the yellow pixel
                Deposit.setPower(1);
                sleep(1500);
                Deposit.setPower(0);
                //drive forwards 1 inch
                GyroDriveENC(1, 0.5, 90, true, true, true, true, 1);
                //bring the kick in and lower the lift for TeleOp
                kick(false);
                liftENC(0, 0.5);
            }

            if(TSEplacement == 1 && rightSide == true){
                //drive forwards 18 IN
                GyroDriveENC(18, 0.5, 0, true, true, true, true, 6);
                //turn left 90 degrees
                GyroSpin(0.5, -90, 15);
                //drive fowards 0.5 IN
                GyroDriveENC(0.5, 0.2, -90, true, true, false, true, 0);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                GyroDriveENC(-2, 0.5, -90, true, false, true, true, -2);

            }

            if (TSEplacement == 2 && rightSide == false){
                //strafe 1 inch to the left
                GyroStrafeENC(1, 0.5, "left", 0);
                //drive foward 24 inches to plow the TSE
                GyroDriveENC(30, 0.5, 0, true, true, true, true, 12);
                sleep(250);
                //drive backwards to give space to deposit the purple pixel
                GyroDriveENC(-2.5, 0.5, 0, true, false, true, true, -2);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                //drive backwards 1 IN
                GyroDriveENC(-1, 0.5, 0, true, false, true, true, -1);
                //raise the arm and extend the kick
                liftENC(600, 0.7);
                kick(true);
                //turn left 90 degrees towards the backdrop
                GyroSpin(0.5, 90, 15);
                //drive backwards towards the backdrop
                GyroDriveENC(-24, 0.5, 90, true, false, true, true, -15);
                //strafe left towards the center of the board
                GyroStrafeENC(4, 0.5, "left", 90);
                //drive closer to the board
                GyroDriveENC(-2, 0.2, 90, true, false, true, true, -2);
                //dispense the yellow pixel
                Deposit.setPower(1);
                sleep(1500);
                Deposit.setPower(0);
                //drive forwards 1 inch
                GyroDriveENC(1, 0.5, 90, true, true, true, true, 1);
                //bring the kick in and lower the lift for TeleOp
                kick(false);
                liftENC(0, 0.5);


            }

            if (TSEplacement == 2 && rightSide == true){
                //strafe 1 inch to the right
                GyroStrafeENC(1, 0.5, "right", 0);
                //drive foward 24 inches to plow the TSE
                GyroDriveENC(30, 0.5, 0, true, true, true, true, 12);
                sleep(250);
                //drive backwards to give space to deposit the purple pixel
                GyroDriveENC(-3.5, 0.5, 0, true, false, true, true, -2);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                GyroDriveENC(-2, 0.5, 0, true, false, true, true, -2);

            }

            if (TSEplacement == 3 && rightSide == false){
                //drive forwards 12 IN
                GyroDriveENC(18, 0.5, 0, true, true, true, true, 6);
                //turn left 90 degrees
                GyroSpin(0.5, 90, 15);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                //raise the lift and extend the kick
                liftENC(600, 0.7);
                kick(true);
                //Drive backwards towards the backdrop
                GyroDriveENC(-24, 0.6, 90, true, false, true, true, -15);
                //strafe to the left 2 inches
                GyroStrafeENC(6, 0.5, "left", 90);
                //drive backwards 1 inch
                GyroDriveENC(-2, 0.2, 90, true, false, true, true, -2);
                //deposit the yellow pixel
                Deposit.setPower(1);
                sleep(1500);
                Deposit.setPower(0);
                //drive forwards 1 inch
                GyroDriveENC(1, 0.5, 90, true, true, true, true, 1);
                //bring the kick in and lower the lift for TeleOp
                kick(false);
                liftENC(0, 0.5);

            }

            if (TSEplacement == 3 && rightSide == true){
                // strafe right 9 IN
                GyroStrafeENC(9, 0.5, "right", 0);
                sleep(150);
                //drive forward 12 IN
                GyroDriveENC(12, 0.5, 0,true, true, true, true, 7);
                //spit out purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                GyroDriveENC(-2, 0.5, 0, true, false, true, true, -2);

            }

        }

        if (redSide){
            if (TSEplacement == 1 && rightSide == true){
                //drive forwards 18 IN
                GyroDriveENC(18, 0.5, 0, true, true, true, true, 6);
                //turn left 90 degrees
                GyroSpin(0.5, -90, 15);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                //raise the lift and extend the kick
                liftENC(600, 0.7);
                kick(true);
                //Drive backwards towards the backdrop
                GyroDriveENC(-24, 0.6, -90, true, false, true, true, -15);
                //strafe to the right 6 inches
                GyroStrafeENC(6, 0.5, "right", -90);
                //drive backwards 0.5 inches
                GyroDriveENC(-0.5, 0.2, -90, true, false, true, true, -2);
                //deposit the yellow pixel
                Deposit.setPower(1);
                sleep(1500);
                Deposit.setPower(0);
                //drive forwards 1 inch
                GyroDriveENC(1, 0.5, -90, true, true, true, true, 1);
                //bring the kick in and lower the lift for TeleOp
                kick(false);
                liftENC(0, 0.5);

            }

            if (TSEplacement == 1 && rightSide == false){
                // strafe left 9 IN
                GyroStrafeENC(9, 0.5, "left", 0);
                sleep(150);
                //drive forward 12 IN
                GyroDriveENC(12, 0.5, 0,true, true, true, true, 7);
                //spit out purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                GyroDriveENC(-2, 0.5, 0, true, false, true, true, -2);
            }

            if (TSEplacement == 2 && rightSide == true){
                //strafe 1 inch to the right
                GyroStrafeENC(1, 0.5, "right", 0);
                //drive foward 24 inches to plow the TSE
                GyroDriveENC(30, 0.5, 0, true, true, true, true, 12);
                sleep(250);
                //drive backwards to give space to deposit the purple pixel
                GyroDriveENC(-3.5, 0.5, 0, true, false, true, true, -2);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                //drive backwards 1 IN
                GyroDriveENC(-1, 0.5, 0, true, false, true, true, -1);
                //raise the arm and extend the kick
                liftENC(600, 0.7);
                kick(true);
                //turn right 90 degrees towards the backdrop
                GyroSpin(0.5, -90, 15);
                //drive backwards towards the backdrop
                GyroDriveENC(-24, 0.5, -90, true, false, true, true, -15);
                //strafe right
                GyroStrafeENC(3.5, 0.5, "right", -90);
                //drive closer to the board
                GyroDriveENC(-2.5, 0.2, -90, true, false, true, true, -2);
                //dispense the yellow pixel
                Deposit.setPower(1);
                sleep(1500);
                Deposit.setPower(0);
                //drive forwards 1 inch
                GyroDriveENC(1, 0.5, -90, true, true, true, true, 1);
                //bring the kick in and lower the lift for TeleOp
                kick(false);
                liftENC(0, 0.5);

            }

            if (TSEplacement == 2 && rightSide == false){
                //strafe 1 inch to the left
                GyroStrafeENC(1, 0.5, "left", 0);
                //drive foward 24 inches to plow the TSE
                GyroDriveENC(30, 0.5, 0, true, true, true, true, 12);
                sleep(250);
                //drive backwards to give space to deposit the purple pixel
                GyroDriveENC(-2.5, 0.5, 0, true, false, true, true, -2);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                GyroDriveENC(-2, 0.5, 0, true, false, true, true, -2);
            }

            if (TSEplacement == 3 && rightSide == true){
                // strafe right 9 IN
                GyroStrafeENC(9, 0.5, "right", 0);
                sleep(150);
                //drive forward 12 IN
                GyroDriveENC(12, 0.5, 0,true, true, true, true, 7);
                //spit out purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                //drive backwards 11 IN
                GyroDriveENC(11, 0.5, 0,false, false, true, true, 3);
                //Turn 90 degrees left
                GyroSpin(0.5, -90,15);
                //raise the lift and extend the kicker
                liftENC(600, 0.75);
                kick(true);
                //drive backwards 12 IN
                GyroDriveENC(-12, 0.5, -90, true, false, true, true, -6);
                //strafe right 12 IN
                GyroStrafeENC(12, 0.5, "right", -90);
                //drive backwards 6 IN
                GyroDriveENC(-6, 0.2, -90, true, false, true, true, -6);
                //drop the yellow pixel
                Deposit.setPower(1);
                sleep(1500);
                Deposit.setPower(0);
                //drive forwards 1 inch
                GyroDriveENC(1, 0.5, -90, true, true, true, true, 1);
                //bring the kick in and lower the lift for TeleOp
                kick(false);
                liftENC(0, 0.5);

            }

            if (TSEplacement == 3 && rightSide == false){
                //drive forwards 12 IN
                GyroDriveENC(18, 0.5, 0, true, true, true, true, 6);
                //turn left 90 degrees
                GyroSpin(0.5, 90, 15);
                //spit out the purple pixel
                Intake_Roller.setPower(-1);
                sleep(2000);
                Intake_Roller.setPower(0);
                GyroDriveENC(-2, 0.5, 90, true, false, true, true, -2);
            }

        }

    }

    //----------------------------------------------------------------------------------------------------
    // FUNCTIONS (MYBLOCKS)
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    //deposit
    public void deposit(boolean intake, double pixelCount){

    }

    //----------------------------------------------------------------------------------------------------
    //kick
    public void kick(boolean kickOut){
        if (kickOut == true){
            kickL.setPosition(0.3);
            kickR.setPosition(1-0.3);
        }
        else {
            kickL.setPosition(0.04);
            kickR.setPosition(1-0.04);
        }
    }

    //----------------------------------------------------------------------------------------------------
    //Lift
    public void liftENC(int position, double power) {

        boolean direction;

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if(position < leftLift.getCurrentPosition()){
            direction = false;
        }
        else {
            direction = true;
        }

        if(direction == false){
            power = -power;
        }

        while((position < leftLift.getCurrentPosition() && direction == false && lowerLimit.getState() == true && (opModeIsActive()))
                ||
                (position > leftLift.getCurrentPosition() && direction == true && leftLift.getCurrentPosition() < 2250 && (opModeIsActive()))){

            leftLift.setPower(power);
            rightLift.setPower(power);
        }

        leftLift.setPower(0);
        rightLift.setPower(0);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    //----------------------------------------------------------------------------------------------------
    // GyroDriveENC
    public void GyroDriveENC(double distance, double power, double course, boolean reset, boolean forward, boolean rampdown, boolean stopmotors, int rampdist) {
        // for telemetry
        desiredCourse = course;


        // if reset is true, then reset the deal wheel encoders
        if (reset == true) {
            intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            climber.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }


        // calcuate the motor power for direction
        power = Math.abs(power);           //this ensures that if we are moving forward it is positive
        if (!forward) {power = -power;}    // and if we are going backwards it is negative

        double originPower = power;

        ReadSensors();
        telemetry.addData("Encoder", intake_reading);
        telemetry.addData("Distance", distance);
        telemetry.addData("OpModeIsActive", opModeIsActive());
        telemetry.addData("forward", forward);
        telemetry.update();
        while ((intake_reading < distance && forward && opModeIsActive())
                || // OR
                (intake_reading > distance && !forward && opModeIsActive())){ //&& opModeIsActive()


            intake_reading = intake.getCurrentPosition()/tickToINCH;



            getRawHeading();
            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

            if (corrHeading < 0) { //robot is drifting to the right, so we need to correct to the left
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power + diffCorrection);
            }
            else if (corrHeading > 0){  //robot is drifting to the left, so we need to correct to the right
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power - diffCorrection);
            }
            else
            {
                leftFrontPower = (power);
                leftRearPower = (-power);
                rightFrontPower = (power);
                rightRearPower = (power);
            }

            if (stopmotors) {
                // ramping down
                delta = Math.abs(Math.abs(distance) - Math.abs(intake_reading));
                int deltaThreshold = rampdist;
                if ((delta <= deltaThreshold) && (forward) && (rampdown)) {
                    if (power > 0.05) {
                        power = (delta / deltaThreshold) * originPower;
                    }
                    //                 else {
                    //                   power = 0.1;
                    //             }

                }

                if ((delta <= deltaThreshold) && (!forward) && (rampdown) && Math.abs(power) > 0.1) {
                    //if (power < -0.2){
                    power = (delta / deltaThreshold) * originPower;

                }
            }

            leftFront.setVelocity(leftFrontPower * driveMaxVelocity);
            leftRear.setVelocity(-leftRearPower * driveMaxVelocity);
            rightFront.setVelocity(rightFrontPower * driveMaxVelocity);
            rightRear.setVelocity(rightRearPower * driveMaxVelocity);

            if (getRuntime() > 29.6){
                leftFront.setVelocity(0);
                leftRear.setVelocity(0);
                rightFront.setVelocity(0);
                rightRear.setVelocity(0);
            }
        }
        if (stopmotors) {


            leftFront.setVelocity(0);
            rightFront.setVelocity(0);
            leftRear.setVelocity(0);
            rightRear.setVelocity(0);
        }

        sleep(150);
    }

    //----------------------------------------------------------------------------------------------------
    // GyroStrafeENC
    public void GyroStrafeENC(double distance, double power, String direction, double course) {
        // for telemetry
        desiredCourse = course;
        ReadSensors();
        distance = Math.abs(distance);
        //direction = toString().toLowerCase(direction);

        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reset the dead wheel encoders
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brake for drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(climber_reading) <= distance && (opModeIsActive())) {
            getRawHeading();
            ReadSensors();
            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

            if ((corrHeading < 0 && direction == "left") || (corrHeading > 0 && direction == "right")) {
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power - diffCorrection);
            }
            else {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power + diffCorrection);
            }

            if(direction == "left") {
                leftFront.setPower(-leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(-rightRearPower);
            }
            else if(direction == "right") {
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(-rightFrontPower);
                rightRear.setPower(rightRearPower);
            }
            else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(150);
    }


    public void ReadSensors() {
        rightFrontEncoder = rightFront.getCurrentPosition();
        leftFrontEncoder = leftFront.getCurrentPosition();
        leftLiftEncoder = leftLift.getCurrentPosition();
        rightLiftEncoder = rightLift.getCurrentPosition();
        climber_reading = climber.getCurrentPosition() / tickToINCH;
        intake_reading = intake.getCurrentPosition() / tickToINCH;
        lowerLimitSwitch = lowerLimit.getState();
        leftFrontVelocity = leftFront.getVelocity();
        leftRearVelocity = leftRear.getVelocity();
        rightFrontVelocity = rightFront.getVelocity();
        rightRearVelocity = rightRear.getVelocity();

    }
    //----------------------------------------------------------------------------------------------------
    // GyroSpin
    public void GyroSpin(double power, double course, double rampdownfactor) {
        // for telemetry
        desiredCourse = course;
        double origpower = power;
        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set break to drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        getRawHeading();

        corrHeading = course - heading;
        diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

        if (heading >= course) {
            while (heading >= course) {
                delta = Math.abs(Math.abs(heading) - Math.abs(course));
                if (delta < rampdownfactor) {
                    power = delta/rampdownfactor*origpower;
                }
                getRawHeading();
                leftFrontPower = (-power);
                leftRearPower = (-power);
                rightFrontPower = (power);
                rightRearPower = (power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);

                if (getRuntime() > 29.6){
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
            }
        }
        else if (heading <= course) {
            while (heading <= course) {
                getRawHeading();
                delta = Math.abs(Math.abs(heading) - Math.abs(course));
                if (delta < rampdownfactor) {
                    power = delta/rampdownfactor*origpower;
                    telemetry.addData("power", power);
                }
                leftFrontPower = (power);
                leftRearPower = (power);
                rightFrontPower = (-power);
                rightRearPower = (-power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);

                if (getRuntime() > 29.6){
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    //----------------------------------------------------------------------------------------------------
    //CAMERA TELEMETRY FUNCTION
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        objects = currentRecognitions.size();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

           if (recognition.getLabel() == "blue 1") {
                TSEplacement = 1;
            }
            else if (recognition.getLabel() == "blue 2") {
                TSEplacement = 2;
            }
            else if (recognition.getLabel() == "red 1") {
                TSEplacement = 1;
            }
            else if (recognition.getLabel() == "red 2") {
                TSEplacement = 2;
            }
            else {
                TSEplacement = 3;
            }

        }// end for() loop

        telemetry.update();

    }   // end method telemetryTfod()

    //----------------------------------------------------------------------------------------------------
    // GYRO FUNCTIONS
    public void getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        heading = -orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    private void sendTelemetry(String codethatsrunning) {
        /*if (SeeTelemetry) {
            telemetry.addData("Luxo Telemetry - Function:", codethatsrunning);
            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", desiredCourse, heading);

            telemetry.addData("Wheel Speeds Front L:R.", "%5.2f : %5.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Wheel Speeds Rear  L:R.", "%5.2f : %5.2f", leftRearPower, rightRearPower);

            telemetry.addData("Actual Pos Para:Perp", "%7.0f:%7.0f", intake_reading, climber_reading);


            // telemetry.addData("rangeSensor", rangeSensor_reading);
            telemetry.addData("Delta:", delta);
            telemetry.addData("Lift ENC T/B, Amps T/B", "%7f : %7f : %5.2f : %5.2f", leftLiftEncoder, rightLiftEncoder, leftLiftAmps, rightLiftAmps);
            telemetry.addData("LiftPower", liftPower);
            telemetry.addData("Velocity - LF, RF, LR, RR:", "%4f : %4f : %4f : %4f", leftFrontVelocity, rightFrontVelocity, leftRearVelocity, rightRearVelocity);
            telemetry.addData("Runtime", getRuntime());

        }
        else
            telemetry.addData("No Telemetry", "For You!");
        telemetry.update();

         */

    }

}

