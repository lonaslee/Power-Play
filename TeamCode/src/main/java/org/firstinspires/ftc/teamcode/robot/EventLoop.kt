package org.firstinspires.ftc.teamcode.robot

import kotlin.reflect.KProperty

class EventLoop(
    private val opModeIsActive: () -> Boolean, private val gamepads: Pair<GamepadExt, GamepadExt>? = null
) {
    fun interface Callback {
        operator fun invoke()
    }

    val updates = mutableListOf<() -> Unit>()

    private val keyEvents = mutableListOf<Pair<() -> Boolean, Callback>>()

    fun onPressed(key: KProperty<*>, func: Callback) {
        keyEvents.add({ pressed(key) } to func)
    }

    fun onAnyPressed(vararg keys: KProperty<*>, func: Callback) {
        keyEvents.add({ keys.any { pressed(it) } } to func)
    }

    fun onReleased(key: KProperty<*>, func: Callback) {
        keyEvents.add({ released(key) } to func)
    }

    fun onAnyReleased(vararg keys: KProperty<*>, func: Callback) {
        keyEvents.add({ keys.any { pressed(it) } } to func)
    }

    fun run() {
        while (opModeIsActive()) {
            if (gamepads != null) keyEvents.filter { it.first() }.forEach { it.second() }

            updates.forEach { it() }
        }
    }
}
