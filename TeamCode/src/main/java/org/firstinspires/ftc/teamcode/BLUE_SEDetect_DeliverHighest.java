package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.List;


@Autonomous(name = "BLUE_SEDetect_DeliverHighest", group = "Linear Opmode")

public class BLUE_SEDetect_DeliverHighest extends AutonomousBot {
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
    private boolean deliverToHighestJunction() {

        leftStrafe(1.3, 0.5);
        moveForward(1.0, 0.5);
        turnLeft(45.00);
        return true;
    }

    @Override
    public void runOpMode() {

        init(hardwareMap);

        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        stdGripperServo.setPosition(1.0);
        sleep(500);
        stdWristServo.setPosition(WRIST_REST_POSITION);

        telemetry.addData("Status", "Initialized");

        /////////////

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

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
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {



            while (opModeIsActive()) {

                scanFieldOfVision();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                            if (recognition.getLabel().equals("SilverEagleSignal1") && recognition.getConfidence() * 100 > 0.750)
                            {
                                deliverToHighestJunction();
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