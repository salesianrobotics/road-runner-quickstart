package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name = "ODO_RIGHT_AUTO1HIGH", group = "Linear Opmode")

public class ODO_RIGHT_AUTO1HIGH extends AutonomousBot {
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


    private void Park1() {
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        rightStrafe(3.35,0.6);
        sleep(300);
       // turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(300);
        moveForward(0.1, 0.5);
        sleep(400);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(400);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
        leftStrafe(0.5, 0.3);
        sleep(200);
        turnRight(191);
        sleep(200);
       // raiseLinearSlide(GROUND_POSITION);
        //stdLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(400);
        raiseLinearSlide(AUTO_CONE_POSITION);
        sleep(300);

        moveForward(1.0, 0.4);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        //leftStrafe(1.1, 0.3);
       // raiseLinearSlide(GROUND_JUNCTION_POSITION);



        // stdWristServo.setPosition(WRIST_UP_POSITION);

    }

    private void Park2() {

        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        rightStrafe(3.35,0.3);
        sleep(300);
        //turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(300);
        moveForward(0.1, 0.5);
        sleep(400);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(400);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
       // moveBackward(0.2, 0.5);
        leftStrafe(0.5, 0.3);
        sleep(200);
        turnRight(191);
        sleep(200);
        // raiseLinearSlide(GROUND_POSITION);
        //stdLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(400);
        raiseLinearSlide(AUTO_CONE_POSITION);
        sleep(300);

        moveForward(1.0, 0.4);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        //leftStrafe(1.1, 0.3);
        // raiseLinearSlide(GROUND_JUNCTION_POSITION);
    }

    private void Park3() {

        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        rightStrafe(3.35,0.3);
        sleep(300);
        //turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(300);
        moveForward(0.1, 0.5);
        sleep(400);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(400);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
        leftStrafe(0.5, 0.3);
        sleep(200);
        turnRight(191);
        sleep(200);
        // raiseLinearSlide(GROUND_POSITION);
        //stdLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(400);
        raiseLinearSlide(AUTO_CONE_POSITION);
        sleep(300);

        moveForward(1.0, 0.4);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        //leftStrafe(1.1, 0.3);
        // raiseLinearSlide(GROUND_JUNCTION_POSITION);
    }

    @Override
    public void runOpMode() {

        init(hardwareMap);

        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(500);
        stdWristServo.setPosition(WRIST_REST_POSITION);

        telemetry.addData("Status", "Initialized");

        /////////////

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
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

        /* Wait for the game to begin */
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
