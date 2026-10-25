package org.spectrum3847.vision.gate;

import java.util.function.Predicate;
import org.spectrum3847.vision.observation.PoseObservation;

/**
 * One check a pose candidate must pass to reach the pose estimator. A gate returns null to pass,
 * or a short reason to reject ("2.9 m > 2.5 m"); its {@link #name()} is what rejections are
 * counted under (see {@link org.spectrum3847.vision.log.VisionStats}).
 *
 * <p>Make your own with {@link #of}, or implement it for a gate with state. Built-in gates are in
 * {@link Gates}; combine them with {@link GateSet}.
 */
public interface Gate {
    String name();

    /** Null to pass, or why not. */
    String reject(PoseObservation obs, GateContext ctx);

    /** Called once per robot loop before any candidate is judged (for gates with state). */
    default void beginLoop(GateContext ctx) {}

    /** Called when a candidate that passed every gate was sent to the pose estimator. */
    default void accepted(PoseObservation obs, GateContext ctx) {}

    static Gate of(String name, Predicate<PoseObservation> passes, String reason) {
        return new Gate() {
            @Override
            public String name() {
                return name;
            }

            @Override
            public String reject(PoseObservation obs, GateContext ctx) {
                return passes.test(obs) ? null : reason;
            }
        };
    }
}
