package org.firstinspires.ftc.teamcode.examples.pasteurized

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.dairy.pasteurized.layering.LayeredGamepad
import dev.frozenmilk.dairy.pasteurized.layering.ListLayeringSystem
import dev.frozenmilk.dairy.pasteurized.layering.MapLayeringSystem
import dev.frozenmilk.dairy.pasteurized.layering.WrappingLayeringSystem

class KotlinOverview : OpMode() {
	override fun init() {

	}

	override fun loop() {
		// Pasteurized offers advanced versions of the gamepads
		Pasteurized.gamepad1
		Pasteurized.gamepad2

		// buttons on the gamepads are represented by EnhancedBooleanSuppliers
		var enhancedBooleanSupplier = Pasteurized.gamepad1.a

		enhancedBooleanSupplier.state // current state
		enhancedBooleanSupplier.onTrue // a rising edge detector
		enhancedBooleanSupplier.onFalse // a falling edge detector
		enhancedBooleanSupplier.toggleTrue // a toggle that gets changed whenever a rising edge is detected
		enhancedBooleanSupplier.toggleFalse // a toggle that gets changed whenever a falling edge is detected

		// EnhancedBooleanSuppliers are immutable by default, so you can pull them out of the gamepad, do one-off modifications to them, and then store and use them again later

		// debouncing can be applied independently to both the rising and falling edge
		// note that each of these operations does not modify the original supplier, attached to gamepad1.a
		enhancedBooleanSupplier = enhancedBooleanSupplier.debounce(0.1)
		enhancedBooleanSupplier = enhancedBooleanSupplier.debounce(0.1, 0.0)
		enhancedBooleanSupplier = enhancedBooleanSupplier.debounceFallingEdge(0.1)
		enhancedBooleanSupplier = enhancedBooleanSupplier.debounceRisingEdge(0.1)

		// if we do not reassign the new EnhancedBooleanSupplier to the variable, or store it in a different variable it will be lost

		// suppliers can also be combined:
		enhancedBooleanSupplier = enhancedBooleanSupplier and { Pasteurized.gamepad1.leftTrigger.state > 5 }
		enhancedBooleanSupplier = enhancedBooleanSupplier or { Pasteurized.gamepad1.leftTrigger.state < 100.0 }

		// this works is all kinds of ways!
		val twoButtons = Pasteurized.gamepad1.a and Pasteurized.gamepad1.b

		// you can also reassign the buttons on the gamepads themselves, if you wish to make a change more global
		Pasteurized.gamepad2.a = Pasteurized.gamepad1.a or Pasteurized.gamepad2.a
		Pasteurized.gamepad1.a = Pasteurized.gamepad2.a
		// now either the driver or the operator can trigger this condition!

		// note: the Pasteurized gamepads have remaps for all gamepad buttons and inputs, the inputs that are shared across the different gamepad types
		// but share a name (i.e. cross on a ps4 controller and a on a logitech or x-box controller) are linked together on the Pasteurized gamepad

		// sticks and triggers are represented via EnhancedNumberSuppliers
		val enhancedNumberSupplier = Pasteurized.gamepad1.leftStickY

		// the value of the stick
		enhancedNumberSupplier.state

		// EnhancedNumberSuppliers also interact well with building complex EnhancedBooleanSuppliers from ranges
		val rangeBasedCondition = enhancedNumberSupplier.conditionalBindState()
				.greaterThan(-0.5)
				.lessThan(0.5)
				.bind()

		// this system is fairly intuitive, and works best if you list numbers from smallest to largest,
		// or in pairs e.g.:

		val complexRangeBasedCondition = enhancedNumberSupplier.conditionalBindState()
				.greaterThan(0.0)
				.lessThan(10.0)
				.greaterThanEqualTo(1.0)
				.lessThanEqualTo(1000.0)
				.bind()


		//
		// Layering
		//
		// WARNING: once you add a gamepad to a layering system, it will cease to function normally
		// when a gamepad is put into a layering system, it is modified to go to being 'at-rest' when its not the active layer for the system
		// 'at-rest' gamepads have 0 for all axis, and false for all buttons

		// If you want to get more advanced, you can use a layering system to control the layers of your gamepad
		// gamepads on inactive layers will immediately go to an 'at-rest' state, layering systems achieve this by modifying all gamepads given to it
		val teleopGamepad = SDKGamepad(gamepad1)
		// note that a PasteurizedGamepad can be constructed from a normal sdk gamepad!
		// this is the same as the original gamepad1 from Pasteurized, but pasteurized will auto generate one for us the first time we ask for it each OpMode
		// WARNING: if your layering system is not very stable, you will need to be careful, as a layered gamepad will crash if it can't find a gamepad for the current layer
		val endgameGamepad = SDKGamepad(gamepad1)

		val pasteurizedGamepadMap = mapOf(
			Layers.TELEOP to teleopGamepad,
			Layers.ENDGAME to endgameGamepad
		)
		val enumLayeringSystem = MapLayeringSystem(Layers.TELEOP, pasteurizedGamepadMap)

		// LayeredGamepad takes a LayeringSystem and makes a gamepad from it
		val layeredGamepad = LayeredGamepad(enumLayeringSystem)

		layeredGamepad.a // a from teleopGamepad
		enumLayeringSystem.layer = Layers.ENDGAME
		layeredGamepad.a // a from endgameGamepad

		// there are many more options for a layering system, and its fairly easy to implement your own!
		// additionally, you can nest layered gamepads in other layered gamepads

		// linking across layers using or
		teleopGamepad.b = teleopGamepad.b.or(endgameGamepad.b)
		endgameGamepad.b = teleopGamepad.b
		// now b on both layers is linked!

		// returns true if this gamepad is active on this layering system
		enumLayeringSystem.isActive(teleopGamepad)

		enumLayeringSystem.layer // current layer
		enumLayeringSystem.gamepad // current gamepad

		//
		// Other Layering Systems
		//
		// the map layering system can be used for any hashable type, but is probably best for enums
		val pasteurizedGamepadMap2: MutableMap<String, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>> =
			HashMap()
		pasteurizedGamepadMap2["one"] = teleopGamepad
		pasteurizedGamepadMap2["two"] = endgameGamepad
		val stringLayeringSystem = MapLayeringSystem("one", pasteurizedGamepadMap2)


		// list layering system can move linearly
		val listLayeringSystem = ListLayeringSystem(
				teleopGamepad,
				endgameGamepad
			)

		// next in the list
		listLayeringSystem.next()
		// previous in the list
		listLayeringSystem.previous()
		// you can't go out of bounds

		// wrapping layering system is similar, but when you reach the end of the bounds, it wraps back to the start
		val wrappingLayeringSystem = WrappingLayeringSystem(
				teleopGamepad,
				endgameGamepad
			)

		// next in the list
		wrappingLayeringSystem.next()
		// we've reached the end, so back to the start
		wrappingLayeringSystem.next()
		// we've reached the start of the list, so back to the end
		wrappingLayeringSystem.previous()
	}

	enum class Layers {
		TELEOP,
		ENDGAME
	}
}