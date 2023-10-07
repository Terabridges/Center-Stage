
package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Hardware example", group="Linear Opmode")

public class hardwareExample extends LinearOpMode {

    Hardware hardware = new Hardware();
    IMUMecDT drivetrain = hardware.getDrivetrain();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);
        drivetrain.setScls(0.2, 0.2);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a)
                drivetrain.setForwards();

            double robotDirection = drivetrain.getDirection();
            double targetDirection = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x);
            double dir = Math.signum(((targetDirection - robotDirection)/Math.PI + 1) % 2 - 1);
            double mag = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

            drivetrain.absoluteDirectionalPow(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Math.min(dir*mag, 1.));
        }
    }
}
