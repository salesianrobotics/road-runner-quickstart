
/********* SILVER EAGLE 2022-2023 FTC POWERPLAY **************/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DuoTeleOpMain", group = "Linear Opmode")
public class DuoTeleOpMain extends StandardBot {

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

    DcMotorImplEx armMotor = null;
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
    public void runOpMode() {

        init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        leftFront = stdLeftFront;
        leftRear = stdLeftRear;
        rightFront = stdRightFront;
        rightRear = stdRightRear;

        linearSlide = stdLinearSlide;
        gripperServo = stdGripperServo;
        wristServo = stdWristServo;

        armMotor = stdArmMotor;

        telemetry.addData("GripperServo", "position (%.2f)", gripperServo.getPosition());
        telemetry.addData("WristServo", "position (%.2f)", wristServo.getPosition());

        telemetry.addData("LinearSlide", "Resetting Encoder");
        linearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        linearSlide.setTargetPositionTolerance(StandardBot.LINEAR_SLIDE_POSITION_TOLERANCE);
        telemetry.addData("LinearSlide", "target position tolerance is %5d", StandardBot.LINEAR_SLIDE_POSITION_TOLERANCE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("LinearSlide", "starting at position %7d", linearSlide.getCurrentPosition());

        telemetry.addData("ArmMotor", "Power is (%.2f)", armMotor.getPower());
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*** START Mechanum Drive Code from Game Manual 0
             *** URL: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html/
             */

            drive = -gamepad1.left_stick_y; // Remember, this is reversed!
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            leftFrontSpeed = (StandardBot.OPTIMAL_DRIVE_SPEED) * (drive + strafe + rotate);
            leftRearSpeed = (StandardBot.OPTIMAL_DRIVE_SPEED) * (drive - strafe + rotate);
            rightFrontSpeed = (StandardBot.OPTIMAL_DRIVE_SPEED) * (drive - strafe - rotate);
            rightRearSpeed = (StandardBot.OPTIMAL_DRIVE_SPEED) * (drive + strafe - rotate);

            /*** END Mechanum Drive Code ***/



            // Send calculated power to wheels
            leftFront.setVelocity(leftFrontSpeed);
            leftRear.setVelocity(leftRearSpeed);
            rightFront.setVelocity(rightFrontSpeed);
            rightRear.setVelocity(rightRearSpeed);

            linearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            linearSlide.setPower(gamepad2.right_stick_y);

            armMotor.setPower(gamepad2.left_stick_y);

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
            checkMotorCurrent("armMotor", armMotor, OPTIMAL_CURRENT_TOLERANCE);
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
            if (gamepad2.dpad_up){
                raiseLinearSlide(HIGH_JUNCTION_POSITION);
            }

            telemetry.addData("WristServo", "position (%.2f)", wristServo.getPosition());

            telemetry.addData("GripperServo", "position (%.2f)", gripperServo.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime);

            //telemetry.addData("LinearSlide", "targetPosition is %7d", linearSlide.getTargetPosition());
            telemetry.addData("LinearSlide", "currentPosition is %7d", linearSlide.getCurrentPosition());
            telemetry.addData("ArmMotor", "Power is (%.2f)", armMotor.getPower());
            telemetry.update();
        }
    }
}
