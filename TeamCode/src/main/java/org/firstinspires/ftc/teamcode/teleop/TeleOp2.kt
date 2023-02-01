package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subsystems.*
import kotlin.system.measureNanoTime

@TeleOp(group = "test")
class TeleOp2 : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm4
    private lateinit var drive: DriveExt

    private val lastGamepad1 = Gamepad()
    private val lastGamepad2 = Gamepad()

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm4(hardwareMap, tm)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)

        waitForStart()
        while (opModeIsActive()) tm.addData("hz", 1 / measureNanoTime {
            arm.state = when {
                gamepad1.dpad_up && !lastGamepad1.dpad_up     -> AnglePresets.next(arm.state)
                gamepad1.dpad_down && !lastGamepad1.dpad_down -> AnglePresets.prev(arm.state)
                gamepad1.a && !lastGamepad1.a                 -> AnglePresets.GROUND
                gamepad1.x && !lastGamepad1.x                 -> AnglePresets.MID
                gamepad1.y && !lastGamepad1.y                 -> AnglePresets.HIGH
                gamepad1.b && !lastGamepad1.b                 -> AnglePresets.BACKHIGH
                else                                          -> arm.state
            }

            if (gamepad1.left_bumper && !lastGamepad1.left_bumper) claw.change()

            drive.update(gamepad1 to gamepad2)
            arm.update()

            lastGamepad1.copy(gamepad1)
            lastGamepad2.copy(gamepad2)
            tm.update()
        } / 1e9)
    }
}
