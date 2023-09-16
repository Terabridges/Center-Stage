
package org.firstinspires.ftc.teamcode.darcytesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Example linear OpMode", group="Linear Opmode")

public class exampleLinearTeleOp extends LinearOpMode {

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "Motor 0");
        motor2 = hardwareMap.get(DcMotor.class, "Motor 1");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("THERE IS TEXT!");
        telemetry.update();

        waitForStart();
        telemetry.addLine("THERE IS MORE TEXT!");
        telemetry.update();


        while (opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(gamepad1.right_stick_y);
            telemetry.addLine("Motor1 pos: " + motor1.getCurrentPosition());
            telemetry.addLine("Motor2 pos: " + motor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
