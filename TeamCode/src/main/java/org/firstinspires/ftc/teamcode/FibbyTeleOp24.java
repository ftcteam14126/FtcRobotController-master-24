package org.firstinspires.ftc.teamcode;

//import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FibbyTeleOp24", group="Iterative OpMode")
public class FibbyTeleOp24 extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //------------ declare motors ---------------------------------------------------------------------------
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private DcMotorEx leftLift = null;
    private DcMotorEx rightLift = null;

    private DcMotor intake = null;
    private DcMotor climber = null;

    //------------- declare servos -----------------------------------------------------------------------
    private Servo kickL;
    private Servo kickR;
    private CRServo Intake_Roller;
    private CRServo Deposit;
    double kickPosition = 0.05;

    double cycles = 0;
    boolean pixelDump = false;

    private Servo launcher;
    double launcherPosition = 0.5;

    //---------- declare sensors -------------------------------------------------------------------------
    DigitalChannel lowerLimit;

    //------------ static variables ---------  ---------------------------------------------------------------

    boolean hangerTime = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //------------ Harware Mapping -------------------------------------------------------------------
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        intake = hardwareMap.get(DcMotor.class, "intake");
        climber = hardwareMap.get(DcMotorEx.class, "climber");

        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");

        kickL = hardwareMap.get(Servo.class,"kickL");
        kickR = hardwareMap.get(Servo.class,"kickR");

        Intake_Roller = hardwareMap.get(CRServo.class,"intakeRoller");
        Deposit = hardwareMap.get(CRServo.class,"deposit");

        launcher = hardwareMap.get(Servo.class,"launcher");

        //----------- Motor Direction ------------------------------------------------------------------------------
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        climber.setDirection(DcMotorSimple.Direction.FORWARD);

        //------------ Reset Encoders ---------------------------------------------------------------------------
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");

        //----------- Constant variables -------------------------------------------------------------------------------

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
//---------------- Driver Controls ------------------------------------------------------------------------------------
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;



        double finesse = 0.35;
        if (gamepad1.right_stick_y != 0) {
            if (gamepad1.left_stick_x < 0) {
                leftFrontPower = -gamepad1.right_stick_y + (gamepad1.left_stick_x * finesse);
                rightFrontPower = -gamepad1.right_stick_y;
                leftBackPower = -gamepad1.right_stick_y + (gamepad1.left_stick_x * finesse);
                rightBackPower = -gamepad1.right_stick_y;
            }
            else if (gamepad1.left_stick_x > 0) {
                leftFrontPower = -gamepad1.right_stick_y;
                rightFrontPower = -gamepad1.right_stick_y - gamepad1.left_stick_x * finesse;
                leftBackPower = -gamepad1.right_stick_y;
                rightBackPower = -gamepad1.right_stick_y - gamepad1.left_stick_x * finesse;
            }
            else {
                leftFrontPower = -gamepad1.right_stick_y;
                rightFrontPower = -gamepad1.right_stick_y;
                leftBackPower = -gamepad1.right_stick_y;
                rightBackPower = -gamepad1.right_stick_y;
            }
        }

        else if (gamepad1.left_stick_x != 0) {
            leftFrontPower = gamepad1.left_stick_x*0.75;
            rightFrontPower = -gamepad1.left_stick_x*0.75;
            leftBackPower = gamepad1.left_stick_x*0.75;
            rightBackPower = -gamepad1.left_stick_x*0.75;
        } else if (gamepad1.left_trigger != 0) { //if left trigger is being pressed it spinning left
            leftFrontPower = -gamepad1.left_trigger;
            rightFrontPower = gamepad1.left_trigger;
            leftBackPower = gamepad1.left_trigger;
            rightBackPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger != 0) {
            leftFrontPower = gamepad1.right_trigger;
            rightFrontPower = -gamepad1.right_trigger;
            leftBackPower = -gamepad1.right_trigger;
            rightBackPower = gamepad1.right_trigger;
        } else {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

       /* telemetry.addData("leftFront","%7d", leftFront.getCurrentPosition());
        telemetry.addData("rightFront","%7d", rightFront.getCurrentPosition());
        telemetry.addData("leftBack","%7d", leftBack.getCurrentPosition());
        telemetry.addData("rightBack","%7d", rightBack.getCurrentPosition());

        */

        //------------ Lift Driver Control -----------------------------------------------------------------------------
        double leftLiftPower;
        double rightLiftPower;
        double currentHeight = leftLift.getCurrentPosition();

        if (gamepad2.right_stick_y < 0 && currentHeight < 2250){
            leftLiftPower = -gamepad2.right_stick_y;
            rightLiftPower = -gamepad2.right_stick_y;
          //  climbPower = -gamepad2.right_stick_y;
        }
         else if (gamepad2.right_stick_y > 0 && lowerLimit.getState() == true){
            leftLiftPower = -gamepad2.right_stick_y;
            rightLiftPower = -gamepad2.right_stick_y;
           // climbPower = -gamepad2.right_stick_y;

        }
        else {
            leftLiftPower = 0;
            rightLiftPower = 0;
        }

        leftLift.setPower(leftLiftPower);
        rightLift.setPower(rightLiftPower);
        //climber.setPower(climbPower * 0.8);

        //------------Climb Control----------------------------------------------------------------
        double climbPower;

        if (gamepad2.left_stick_y < 0 && lowerLimit.getState() == true) {
            climbPower = gamepad2.left_stick_y;
        }
        else if (gamepad2.left_stick_y > 0) {
            climbPower = gamepad2.left_stick_y;
            }
        else {
            climbPower = 0;
        }

        climber.setPower(climbPower);

        //----------- Kick Control ----------------------------------------------------------------

        //turns on hangertime if button a is pressed
        if (gamepad2.a){
            hangerTime = true;
        }

        //if the lift is going up, activate the kicker
        if (currentHeight > 575 && hangerTime == false) {
            kickPosition = 0.3;
        }
        //if the lift is going down, retract the kicker
        else if (currentHeight <= 575 && hangerTime == false) {
            kickPosition = 0.04;
        }

        //if we want to hang, activate the kicker
        else if(hangerTime == true){
            kickPosition = 0.32;
        }
        kickL.setPosition(kickPosition);
        kickR.setPosition(1-kickPosition);


        //telemetry.addData("Kick Servo Position", "%5.2f", kickPosition);
       // telemetry.update();

        //------------ Intake Driver Control -----------------------------------------------------------------------------
        if (gamepad2.right_bumper && lowerLimit.getState() == false)
        {
            Intake_Roller.setPower(1);
            Deposit.setPower(-1);
            intake.setPower(0.5);
        }
        else if (gamepad2.left_bumper) {
            Deposit.setPower(1);
            Intake_Roller.setPower(0);
            intake.setPower(0);
        }
        else if (gamepad2.left_trigger > 0 && lowerLimit.getState() == false){
            Intake_Roller.setPower(-1);
            Deposit.setPower(0);
            intake.setPower(-0.5);
        }
        else {
            Intake_Roller.setPower(0);
            intake.setPower(0);
            Deposit.setPower(0);
        }

        if (gamepad2.dpad_up){
            pixelDump = true;
            cycles = 0;
        }

        if (pixelDump == true){
            Deposit.setPower(1);
            if (cycles >= 6){
                Deposit.setPower(0);
                pixelDump = false;
            }
        }



        //----------- Launcher -------------------------------------------------------------------
        if (gamepad2.right_stick_button) {
            launcherPosition =0.75;
        }

        launcher.setPosition(launcherPosition);

        //----------- Telemetry ----------------------------------------------------------------
     //   telemetry.addData("Lift Position","%7d", leftLift.getCurrentPosition());
     //   telemetry.addData("lower Limit", lowerLimit.getState());



        //------- ODOMETRY TEST---1.01419878;------------------------------------------

        //ticks per inch parallel (vertical) multiplier

        //ticks per inch perpendicular multiplier
        double TPI = 1.03092784;

        double paraL = (intake.getCurrentPosition()/345)*TPI;
        double paraR = (rightLift.getCurrentPosition()/345)*TPI;
        double perp = (climber.getCurrentPosition()/345)*TPI;

       // telemetry.addData("Actual Pos left:right:Perp",  "%5.2f : %5.2f : %5.2f", paraL, paraR, perp);

        cycles = cycles + 1;
    }

    @Override
    public void stop() {
    }

}