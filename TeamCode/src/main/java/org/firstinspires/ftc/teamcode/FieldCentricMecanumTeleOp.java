package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentricMecanumTeleOp", group = "Linear Opmode")
public class FieldCentricMecanumTeleOp extends StandardBot
{

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    //private final double armPower = 0;
    //private final double carouselPower = 0;
    /* Declare OpMode members. */
    //StandardBot robot = new StandardBot();   // Use a StandardBot's hardware

    DcMotorImplEx leftFront = null;
    DcMotorImplEx leftRear = null;
    DcMotorImplEx rightFront = null;
    DcMotorImplEx rightRear = null;

    DcMotorImplEx linearSlide = null;

    Servo gripperServo = null;
    Servo wristServo = null;

    double leftFrontSpeed = 0;
    double leftRearSpeed = 0;
    double rightFrontSpeed = 0;
    double rightRearSpeed = 0;

    int linearSlidePosition = 0;

    double drive = 0;
    double strafe = 0;
    double rotate = 0;
    double contPower;

    final double MAX_POS     = 0.80;     // Maximum rotational position
    final double MIN_POS     = 0.32;     // Minimum rotational position

    private void doubleLeftStrafe()
    {
        leftStrafe(2.0,1.7);

        //raiseLinearSlide(HIGH_JUNCTION_POSITION);

        setDriveTrainToRunWithoutEncoder();

    }

    private void doubleRightStrafe()
    {
        rightStrafe(2.0,1.7);
        setDriveTrainToRunWithoutEncoder();

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declare our motors
        // Make sure your ID's match your configuration
        init(hardwareMap);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        imu.initialize(parameters);


        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        leftFront = stdLeftFront;
        leftRear = stdLeftRear;
        rightFront = stdRightFront;
        rightRear = stdRightRear;

        linearSlide = stdLinearSlide;
        gripperServo = stdGripperServo;
        wristServo = stdWristServo;

        telemetry.addData("GripperServo", "position (%.2f)", gripperServo.getPosition());
        telemetry.addData("WristServo", "position (%.2f)", wristServo.getPosition());

        telemetry.addData("LinearSlide", "Resetting Encoder");
        linearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        linearSlide.setTargetPositionTolerance(StandardBot.LINEAR_SLIDE_POSITION_TOLERANCE);
        telemetry.addData("LinearSlide", "target position tolerance is %5d", StandardBot.LINEAR_SLIDE_POSITION_TOLERANCE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("LinearSlide", "starting at position %7d", linearSlide.getCurrentPosition());

        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing


            double rx = gamepad1.right_stick_x;

            // Check to see if Yaw reset is requested (Y button)
            if (gamepad1.y)
            {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            }
            else
            {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset.\n");
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Heading (RAD)", botHeading );

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            telemetry.addData("FrontLeftPower", frontLeftPower);
            telemetry.addData("BackLeftPower", backLeftPower);
            telemetry.addData("FrontRightPower", frontRightPower);
            telemetry.addData("BackRightPower", backRightPower);

            // Create an object to receive the IMU angles
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();

// Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            telemetry.addData("Yaw (DEG)", Yaw);
            telemetry.addData("Pitch (DEG)", Pitch);
            telemetry.addData("Roll (DEG)", Roll);


            linearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide.setPower(gamepad2.right_stick_y);

            //        if (gamepad2.right_stick_y != 0)
            //        {
            //            linearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //            linearSlide.setPower(gamepad2.right_stick_y);
            //        } else {

            // manually resets the encoder
            if (gamepad2.right_stick_button){
                linearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }

            // default heights for different junctions
            //if(gamepad2.dpad_down){
            //    raiseLinearSlide(GROUND_JUNCTION_POSITION);
            //}
            if(gamepad1.dpad_right) {
                doubleRightStrafe();
            }
            else if(gamepad1.dpad_left){
                doubleLeftStrafe();
            }

            //      }



            telemetry.addData("DriveTrain", "leftFront (%.2f), rightFront (%.2f)", leftFrontSpeed, rightFrontSpeed);
            telemetry.addData("DriveTrain", "leftRear (%.2f), rightRear (%.2f)", leftRearSpeed, rightRearSpeed);
            //telemetry.addData("LinearSlide", "speed (%.2f)", linearSlidePosition);

            checkMotorCurrent("leftFrontMotor", leftFront, OPTIMAL_CURRENT_TOLERANCE);
            checkMotorCurrent("leftRearMotor", leftRear, OPTIMAL_CURRENT_TOLERANCE);
            checkMotorCurrent("rightFrontMotor", rightFront, OPTIMAL_CURRENT_TOLERANCE);
            checkMotorCurrent("rightRearMotor", rightRear, OPTIMAL_CURRENT_TOLERANCE);

            checkMotorCurrent("linearSlideMotor", linearSlide, LINEAR_SLIDE_OPTIMAL_CURRENT_TOLERANCE);

            // Gripper servo
            if(gamepad2.left_bumper) // OPEN GRIPPER
            {
                gripperServo.setPosition(GRIPPER_OPENED_POSITION);

            }
            else if(gamepad2.right_bumper) // CLOSE GRIPPER
            {
                gripperServo.setPosition(GRIPPER_CLOSED_POSITION);
            }

            // Wrist Servo
            if(gamepad2.y) //wrist up
            {
                stdWristServo.setPosition(WRIST_UP_POSITION);
            }
            else if (gamepad2.a) // wrist down
            {
                stdGripperServo.setPosition(GRIPPER_MIDDLE_POSITION);

                wristServo.setPosition(WRIST_DOWN_POSITION);
            }
            else if (gamepad2.x)
            {
                wristServo.setPosition(WRIST_MIDDLE_POSITION);
            }
            else if (gamepad2.b)
            {
                wristServo.setPosition(WRIST_REST_POSITION);
            }


            telemetry.addData("WristServo", "position (%.2f)", wristServo.getPosition());

            telemetry.addData("GripperServo", "position (%.2f)", gripperServo.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime);

            //telemetry.addData("LinearSlide", "targetPosition is %7d", linearSlide.getTargetPosition());
            telemetry.addData("LinearSlide", "currentPosition is %7d", linearSlide.getCurrentPosition());

            telemetry.update();
        }

    }

}