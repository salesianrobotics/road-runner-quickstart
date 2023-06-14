package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name = "RIGHT_AUTO2HIGH_ODOMETRY", group = "Linear Opmode")

public class RIGHT_AUTO2HIGH_ODOMETRY extends AutonomousBot {
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
    int autoConePosition = AUTO_CONE_POSITION;

    private void Park1() {
        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        //raiseLinearSlide(HIGH_JUNCTION_POSITION);
        rightStrafe(3.25,0.6);
        //leftStrafe(0.25,0.6);
        // turnRight(45);
        //turnRight(47.5);
        //moveForward(0.2,0.6);
        //correctPathHeading(localizer.getPoseEstimate(), new Pose2d(42, 0,0));
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        moveForward(0.1,0.6);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(200);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(200);
        moveBackward(0.1,0.6);
        leftStrafe(0.3,0.6);
        for(int i = 0; i < 1; i++)
        {
            raiseLinearSlide(autoConePosition);
            turnLeft(185);
            //raiseLinearSlide(autoConePosition);
            //turnRight(147);
            moveForward(1.2,0.6);
            stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
            sleep(200);
            raiseLinearSlide(HIGH_JUNCTION_POSITION);
            moveBackward(1.2, 0.6);
            turnLeft(150);
            moveForward(0.2,0.8);
            stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
            sleep(200);
            stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
            sleep(200);
            autoConePosition -= AUTO_CONE_POSITION_DECREMENT;
        }
        moveBackward(0.2,0.6);
        raiseLinearSlide(0);
        turnLeft(47.5);
        moveForward(0.75,1.0);

    }

    private void Park2() {

        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        //raiseLinearSlide(HIGH_JUNCTION_POSITION);
        rightStrafe(3.2,0.6);
        //leftStrafe(0.25,0.6);
        // turnRight(45);
        //turnRight(47.5);
        //moveForward(0.2,0.6);

        correctPathHeading(localizer.getPoseEstimate(), new Pose2d(42, 0,0));

        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        moveForward(0.1,0.6);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(200);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(200);
        moveBackward(0.1,0.6);
        leftStrafe(0.3,0.6);
        for(int i = 0; i < 1; i++)
        {
            raiseLinearSlide(autoConePosition);
            turnLeft(185);
            //raiseLinearSlide(autoConePosition);
            //turnRight(147);
            moveForward(1.2,0.6);
            stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
            sleep(200);
            raiseLinearSlide(HIGH_JUNCTION_POSITION);
            moveBackward(1.2, 0.6);
            turnLeft(150);
            moveForward(0.2,0.8);
            stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
            sleep(200);
            stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
            sleep(200);
            autoConePosition -= AUTO_CONE_POSITION_DECREMENT;
        }
        moveBackward(0.2,0.6);
        raiseLinearSlide(0);
    }

    private void Park3() {

        Pose2d currentPose = localizer.getPoseEstimate();
        telemetry.addData("Current Odometer Readings", "(%.2f, %.2f, %.2f)", currentPose.getX(), currentPose.getY(), currentPose.getHeading());

        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        //raiseLinearSlide(HIGH_JUNCTION_POSITION);
        rightStrafe(3.2,0.6);

        currentPose = localizer.getPoseEstimate();
        telemetry.addData("Current Odometer Readings", "(%.2f, %.2f, %.2f)", currentPose.getX(), currentPose.getY(), currentPose.getHeading());

        //leftStrafe(0.25,0.6);
        // turnRight(45);
        //turnRight(47.5);
        //moveForward(0.2,0.6);
        correctPathHeading(currentPose, new Pose2d(42, 0,0));

        telemetry.update();
        
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        moveForward(0.1,0.6);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(200);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(200);
        moveBackward(0.1,0.6);
        leftStrafe(0.3,0.6);
        for(int i = 0; i < 1; i++)
        {
            raiseLinearSlide(autoConePosition);
            turnLeft(185);
            //raiseLinearSlide(autoConePosition);
            //turnRight(147);
            moveForward(1.2,0.6);
            stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
            sleep(200);
            raiseLinearSlide(HIGH_JUNCTION_POSITION);
            moveBackward(1.2, 0.6);
            turnLeft(150);
            moveForward(0.2,0.8);
            stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
            sleep(200);
            stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
            sleep(200);
            autoConePosition -= AUTO_CONE_POSITION_DECREMENT;
        }
        moveBackward(0.2,0.6);
        raiseLinearSlide(0);
        turnLeft(47.5);
        moveBackward(0.75,1.0);

    }

    @Override
    public void runOpMode() {

        init(hardwareMap);

        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(1000);
        stdWristServo.setPosition(WRIST_REST_POSITION);

        telemetry.addData("Status", "Initialized");

        /////////////

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        initLocalizer();

        Pose2d currentPose = localizer.getPoseEstimate();
        telemetry.addData("Current Odometer Readings", "(%.2f, %.2f, %.2f)", currentPose.getX(), currentPose.getY(), currentPose.getHeading());


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {


            while (opModeIsActive()) {


                //scanFieldOfVision();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            // WRITE CODE WHAT TO DO WHEN DETECTED SIGNAL SLEEVE

                            if (recognition.getLabel().equals("SilverEagleSignal1") && recognition.getConfidence() * 100 > 0.750) {
                                Park1();
                                return;
                            } else if (recognition.getLabel().equals("SilverEagleSignal2") && recognition.getConfidence() * 100 > 0.750) {
                                Park2();
                                return;
                            } else if (recognition.getLabel().equals("SilverEagleSignal3") && recognition.getConfidence() * 100 > 0.750) {
                                Park3();
                                return;
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }
}
