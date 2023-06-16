package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LinearSlideControl {

    private DcMotorImplEx stdLinearSlide = null; // motor to control the linear slide

    public LinearSlideControl(HardwareMap hwMap, String slideLabel)
    {
        stdLinearSlide = hwMap.get(DcMotorImplEx.class, slideLabel);
        stdLinearSlide.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);
    }

    public void raise(int position) throws InterruptedException
    {


        //telemetry.addData("raiseLinearSlide", "Setting to RUN WITH ENCODER");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stdLinearSlide.setDirection(DcMotorEx.Direction.FORWARD);

        //telemetry.addData("raiseLinearSlide", "Setting TargetPosition to %5d", position);
        stdLinearSlide.setTargetPosition(position);

        //telemetry.addData("raiseLinearSlide", "Setting Run to Position");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Can't hurt to call this repeatedly

        //telemetry.addData("raiseLinearSlide", "Setting Arm Velocity to %7.2f", OPTIMAL_ARM_SPEED );
        stdLinearSlide.setVelocity(StandardBot.OPTIMAL_LINEAR_SLIDE_SPEED);

        while (stdLinearSlide.isBusy() && !stdLinearSlide.isOverCurrent())
        {
            wait(10);
            //   telemetry.addData("raiseLinearSlide", "targetPosition is %7d", stdLinearSlide.getTargetPosition());
            //   telemetry.addData("raiseLinearSlide", "currentPosition is %7d", stdLinearSlide.getCurrentPosition());
            //   telemetry.addData("raiseLinearSlide", "CURRENT is %5.2f milli-amps", stdLinearSlide.getCurrent(CurrentUnit.MILLIAMPS));

            //   telemetry.update();
        }

        //telemetry.addData("raiseLinearSlide", "Setting velocity to 0.0 now");
        stdLinearSlide.setVelocity(0.0);

    }

    public void lower(int position) throws InterruptedException {


        //telemetry.addData("raiseLinearSlide", "Setting to RUN WITH ENCODER");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stdLinearSlide.setDirection(DcMotorEx.Direction.REVERSE);

        //telemetry.addData("raiseLinearSlide", "Setting TargetPosition to %5d", position);
        stdLinearSlide.setTargetPosition(position);

        //telemetry.addData("raiseLinearSlide", "Setting Run to Position");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Can't hurt to call this repeatedly

        //telemetry.addData("raiseLinearSlide", "Setting Arm Velocity to %7.2f", OPTIMAL_ARM_SPEED );
        stdLinearSlide.setVelocity(StandardBot.OPTIMAL_LINEAR_SLIDE_SPEED);

        while (stdLinearSlide.isBusy() && !stdLinearSlide.isOverCurrent())
        {
            wait(10);
            //   telemetry.addData("raiseLinearSlide", "targetPosition is %7d", stdLinearSlide.getTargetPosition());
            //   telemetry.addData("raiseLinearSlide", "currentPosition is %7d", stdLinearSlide.getCurrentPosition());
            //   telemetry.addData("raiseLinearSlide", "CURRENT is %5.2f milli-amps", stdLinearSlide.getCurrent(CurrentUnit.MILLIAMPS));

            //   telemetry.update();
        }

        //telemetry.addData("raiseLinearSlide", "Setting velocity to 0.0 now");
        stdLinearSlide.setVelocity(0.0);

    }

}
