package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.jvm.internal.CallableReference
import kotlin.reflect.KProperty
import kotlin.reflect.jvm.javaField

/**
 * Subclass wrapper of [Gamepad] with helper functions
 */
class GamepadExt(private val gamepad: Gamepad) : Gamepad() {
    val prev = Gamepad()

    fun sync() = prev.copy(this).also { this.copy(gamepad) }
}

/* * * * * * * * * * * *\
 * GamepadExt helpers  *
\* * * * * * * * * * * */

typealias Gamepads = Pair<GamepadExt, GamepadExt>

fun Gamepads.sync() {
    first.sync()
    second.sync()
}

fun pressed(key: KProperty<*>): Boolean {
    val gamepad = (key as CallableReference).boundReceiver as GamepadExt
    return key.javaField!!.let { !it.getBoolean(gamepad.prev) && it.getBoolean(gamepad) }
}

fun released(key: KProperty<*>): Boolean {
    val gamepad = (key as CallableReference).boundReceiver as GamepadExt
    return key.javaField!!.let { it.getBoolean(gamepad.prev) && !it.getBoolean(gamepad) }
}

fun anypressed(vararg keys: KProperty<*>) = keys.any { pressed(it) }

fun anyreleased(vararg keys: KProperty<*>) = keys.any { pressed(it) }
