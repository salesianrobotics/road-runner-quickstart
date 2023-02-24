package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.List;


@Autonomous(name = "AutoDriveTest", group = "Linear Opmode")

public class AutoDriveTest extends AutonomousBot {
    private Blinker control_Hub;
    private Blinker expansion_Hub;
    private Servo gripperServo;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor linearSlide;
    private HardwareDevice logitechWebcam;
    private ColorSensor rEVColorSensorV3;
    private DistanceSensor rev2MDistanceSensor;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private Servo wristServo;

    // delivers to highest junction and returns true if successful

    private void turn360Right(){

        turnRight(90);
        sleep(500);
        turnRight(90);
        sleep(500);
        turnRight(90);
        sleep(500);
        turnRight(90);
        sleep(500);

    }
    private void turn360Left()   {
        turnLeft(90);
        sleep(500);
        turnLeft(90);
        sleep(500);
        turnLeft(90);
        sleep(500);
        leftTurn(90);
        sleep(500);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(500);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(500);
    }
    private void driveForward(double nTiles) {
        moveForward(nTiles);

    }

    private void driveBackward(double nTiles) {
        moveBackward(nTiles);

    }

    private void strafeLeft(double nTiles) {

        leftStrafe(nTiles);
    }

    private void strafeRight(double nTiles) {

        rightStrafe(nTiles + RIGHT_STRAFE_ADJUSTMENT);
    }

    private void rightTurn(double angleInDegrees) {

        turnRight(angleInDegrees);
    }

    private void leftTurn(double angleInDegrees) {

        turnLeft(angleInDegrees);
    }




    @Override
    public void runOpMode() {

        init(hardwareMap);

        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(500);
        stdWristServo.setPosition(WRIST_REST_POSITION);

        telemetry.addData("Status", "Initialized");


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            //leftStrafe(1.0); //works
            // turnLeft(90); //works
            // turnRight(90); //works
            //moveForward(1.0); // works
            //moveBackward(1.0);
            //ightStrafe(1.0)
            //stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
            raiseLinearSlide(HIGH_JUNCTION_POSITION);
            sleep(200);
            stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
            sleep(10000);


            // driveBackward(2);
        }
    }
}
