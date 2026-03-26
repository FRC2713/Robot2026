package frc2713.lib.logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Marks a periodic method to be timed and logged through AdvantageKit. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface TimeLogged {
  /** AdvantageKit output path root. */
  String value();
}
