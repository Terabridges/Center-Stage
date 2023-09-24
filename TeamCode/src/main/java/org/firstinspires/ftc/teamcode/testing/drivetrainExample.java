
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Drivetrain example", group="Linear Opmode")

public class drivetrainExample extends LinearOpMode {

    Drivetrain drivetrain = new Drivetrain();
    IMU imu;
    double forwards = 0;

//    double xVel = 0;
//    double yVel = 0;
//    double accel = 0;
//    double

    @Override
    public void runOpMode() {

        drivetrain.setMotors(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        while (opModeIsActive()) {
            double robotDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("", robotDirection - forwards);
            if(gamepad1.a)
                forwards = robotDirection;

            double targetDirection = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x);
            double diff = (targetDirection - robotDirection + Math.PI) / Math.PI % 2 - 1;

            drivetrain.absoluteTransRotPow(robotDirection - forwards, -gamepad1.left_stick_y, -gamepad1.left_stick_x, Math.max(diff, 1.));

//            telemetry.addLine("Motor1 pos: " + motor1.getCurrentPosition());
//            telemetry.addLine("Motor2 pos: " + motor2.getCurrentPosition());
//            telemetry.addLine("imu: " + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


            telemetry.update();
        }
    }
}
