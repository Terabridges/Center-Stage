package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        DcMotor viperSlideLeft = hardwareMap.get(DcMotor.class, "viper_slide_left");
        viperSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor viperSlideRight = hardwareMap.get(DcMotor.class, "viper_slide_right");
        viperSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo elbow_servo = hardwareMap.get(Servo.class, "elbow_servo" );
        Servo claw_servo = hardwareMap.get(Servo.class, "claw_servo" );

        //Values will change, need to find position in which arm is reset back to "zero" position (ready to pick up pixels)
        int slideZeroPos = viperSlideLeft.getCurrentPosition();
        double elbowZeroPos = elbow_servo.getPosition();
        double clawZeroPos = claw_servo.getPosition();

        //Important: 0 is temporary, test to find best position to place pixels
        int slidePlacePos = 0;
        double elbowPlacePos = 0;
        double clawPlacePos = 0;

        waitForStart();
        while(opModeIsActive())
        {
            //If needed, hold left bumper to move freely
            if (gamepad1.left_bumper)
            {
                viperSlideLeft.setPower(gamepad1.left_stick_y);
                viperSlideRight.setPower(gamepad1.left_stick_y);
                //Note: Make one for elbow and claw servos too.

            }
            //press x to reset
            if (gamepad1.x)
            {
                viperSlideLeft.setTargetPosition(slideZeroPos);
                viperSlideRight.setTargetPosition(slideZeroPos);
                elbow_servo.setPosition(elbowZeroPos);
                claw_servo.setPosition(clawZeroPos);
            }
            //press y to move to position to place pixels
            if (gamepad1.a)
            {
                viperSlideLeft.setTargetPosition(slidePlacePos);
                viperSlideRight.setTargetPosition(slidePlacePos);
                elbow_servo.setPosition(elbowPlacePos);
                claw_servo.setPosition(clawPlacePos);
            }
        }
    }

}