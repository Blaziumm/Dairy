//DOUBLE COMPONENT MUST BE IN YOUR TEAMCODE FOLDER, IT CAN BE FOUND AT DAIRY/TeamCode/teamcode/examples/controller/DoubleComponent

package org.firstinspires.ftc.teamcode.examples.controller;

import static org.firstinspires.ftc.teamcode.util.Constants.Slides.*;
import org.firstinspires.ftc.teamcode.DoubleComponent;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.controller.implementation.DoubleController;
import dev.frozenmilk.dairy.core.util.supplier.numeric.CachedMotionComponentSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.util.cell.Cell;

public class SlidesPIDExample extends SDKSubsystem {
    public static final SlidesPIDExample INSTANCE = new SlidesPIDExample();
    private SlidesPIDExample() {}

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach{}

    //Dependency Setup
    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    //Slide State Setup
    public enum SlideState {

        FULL_EXTEND {
            @Override
            public void apply() {
                SlidesPIDExample.INSTANCE.setSlidePosition(fullIntakeExtend);

            }
        },
        HALF_EXTEND {
            @Override
            public void apply() {
                SlidesPIDExample.INSTANCE.setSlidePosition(halfIntakeExtend);

            }
        },
        HOME {
            @Override
            public void apply() {
                SlidesPIDExample.INSTANCE.setSlidePosition(homePos);
            }
        };
        public abstract void apply();
    }

    //Controller Tolerance Setup
    private double SlidesPos = 0.0;
    private double posTolerance = 5.0;
    private double velTolerance = 1.0;

    //Controller Coefficients Setup

    private double SlidesP = 0.0
    private double SlidesI = 0.0
    private double SlidesD = 0.0

    private String HORIZONTALRIGHT = "right";
    private String HORIZONTALLEFT = "left";


    public static SlideState slideState;

    //Hardware Initialization
    private final Cell<DcMotorEx> rightslides = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, HORIZONTALRIGHT));
    private final Cell<DcMotorEx> leftslides = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, HORIZONTALLEFT));

    //Cell Initialization
    private final Cell<EnhancedDoubleSupplier> encoder = subsystemCell(() -> new EnhancedDoubleSupplier(() -> (double) rightslides.get().getCurrentPosition()));
    private final Cell<EnhancedDoubleSupplier> current = subsystemCell(() -> new EnhancedDoubleSupplier(() -> rightslides.get().getCurrent(CurrentUnit.AMPS)));
    private final CachedMotionComponentSupplier<Double> targetSupplier = new CachedMotionComponentSupplier<>(motionComponent -> {
        if (motionComponent == MotionComponents.STATE) {
            return SlidesPos;
        }
        return Double.NaN;
    });
    private final CachedMotionComponentSupplier<Double> toleranceSupplier = new CachedMotionComponentSupplier<>(motionComponent -> {
        if (motionComponent == MotionComponents.STATE) {
            return posTolerance;
        }
        return Double.NaN;
    });


    //Controller Definition
    private final Cell<DoubleController> controller = subsystemCell(() ->
            new DoubleController(
                    targetSupplier,
                    encoder.get(),
                    toleranceSupplier,
                    (Double power) -> {
                        rightslides.get().setPower(power);
                        leftslides.get().setPower(power);
                    },
                    new DoubleComponent.P(MotionComponents.STATE, () -> SlidesP)
                            .plus(new DoubleComponent.I(MotionComponents.STATE, () -> SlidesI))
                            .plus(new DoubleComponent.D(MotionComponents.STATE, () -> SlidesD))
            )
    );

    //Wrapper Definition
    private Wrapper opmodeWrapper;

    //Hooks
    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        rightslides.get().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        opmodeWrapper = opMode;
        controller.get().setEnabled(false);
    }
    @Override
    public void preUserStartHook(@NonNull Wrapper opMode) {


        controller.get().setEnabled(true);
    }

    //Utility Functions
    public void setTarget(double target) {
        controller.get().setEnabled(true);
        this.SlidesPos = target;
        targetSupplier.reset();
    }

    public void retract() {
        controller.get().setEnabled(false);
        rightslides.get().setPower(-1);
    }

    public void stopSlides() {
        rightslides.get().setPower(0);
        setTarget(0);
    }

    public void setSlides(SlideState slideState) {
        SlidesPIDExample.slideState = slideState;
        slideState.apply();
    }

    public void resetEncoder() {
        rightslides.get().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetSupplier.reset();
    }

    //Return Functions
    public double getVelocity() {
        return (this.encoder.get().rawVelocity());
    }

    public double getCurrent() {
        return (current.get().state());
    }

    public double getCurrentChange() {
        return (current.get().rawVelocity());
    }

    public double getEncoder() {
        return (encoder.get().state());
    }

    public boolean getControllerFinished() {
        return (controller.get().finished());
    }

    public double getHorizontalSlidesPos() {
        return SlidesPos;
    }

    //Lambda Commands
    public Lambda setSlidePosition(double target) {
        return new Lambda("set-slide-position-double-horizontal")
                .setInit(() -> setTarget(target))
               ;// .setFinish(() -> controller.get().finished() || opmodeWrapper.getState() == Wrapper.OpModeState.STOPPED);
    }
    public Lambda setSlidePosition(SlideState slideState) {
        return new Lambda("set-slide-position-state-horizontal")
                .setInit(() -> setSlides(slideState))
                .setFinish(() -> controller.get().finished() || opmodeWrapper.getState() == Wrapper.OpModeState.STOPPED);
    }

}
