package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Arm3(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val topmotor = hardwareMap[Config.TOP_LIFT.s] as DcMotorEx
    private val lowmotor = hardwareMap[Config.LOW_LIFT.s] as DcMotorEx
    private val motors = listOf(topmotor, lowmotor).onEach {
        it.direction = DcMotorSimple.Direction.REVERSE
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        it.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private val upControl = PIDFController(kP, kI, kD, kF)
    private val downControl = PIDFController(dkP, dkI, dkD, dkF)

    var height = Height.FLR
    private var goingDown = false

    fun up() {
        goingDown = false
        height = height.next
    }

    fun down() {
        goingDown = true
        height = height.prev
    }

    fun update() {
        println("HEIGHT : ${height.name}, ${height.pos};   GOINGDOWN: $goingDown")

        val armPos = lowmotor.currentPosition
        val pow = (if (goingDown) downControl else upControl).calculate(
            armPos.toDouble(), height.pos.toDouble()
        )
        println()

        motors.forEach { it.power = pow }

        telemetry.addData("pos", armPos)
        telemetry.addData("ref", height.pos)
        telemetry.addData("pow", pow)
    }

    fun adjustAccordingTo(gp1: GamepadExt, gp2: GamepadExt) {
        if (gp1 pressed gp1::y || gp2 pressed gp2::y) up()
        else if (gp1 pressed gp1::a || gp2 pressed gp2::a) down()
        update()
    }

    infix fun adjustAccordingTo(gps: Pair<GamepadExt, GamepadExt>) =
        adjustAccordingTo(gps.component1(), gps.component2())

    companion object {
        var kP = 0.0026
        var kI = 0.0
        var kD = 0.00001
        var kF = 0.0

        var dkP = 0.001
        var dkI = 0.0
        var dkD = 0.00001
        var dkF = 0.0
    }

    enum class Height(val pos: Int) {
        TOP(240), MID(240), LOW(240), FLR(5);

        val next
            get() = when (this) {
                FLR -> LOW
                else -> MID
            }
        val prev
            get() = when (this) {
                TOP -> MID
                MID -> LOW
                else -> FLR
            }
    }
}
