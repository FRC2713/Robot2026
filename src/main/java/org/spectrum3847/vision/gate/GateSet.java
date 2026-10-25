package org.spectrum3847.vision.gate;

import java.util.ArrayList;
import java.util.List;
import org.spectrum3847.vision.observation.PoseObservation;

/**
 * Gates in order. A candidate is judged by each until one rejects it: the first rejection is the
 * one recorded. Immutable: {@link #with}, {@link #without} and {@link #replace} make new sets, so a
 * team can start from {@link Gates#recommended()} and change one thing.
 */
public final class GateSet {
    private final List<Gate> gates;

    private GateSet(List<Gate> gates) {
        this.gates = List.copyOf(gates);
    }

    public static GateSet of(Gate... gates) {
        return new GateSet(List.of(gates));
    }

    public static GateSet none() {
        return new GateSet(List.of());
    }

    public List<Gate> gates() {
        return gates;
    }

    /** These gates added at the end. */
    public GateSet with(Gate... more) {
        var l = new ArrayList<>(gates);
        l.addAll(List.of(more));
        return new GateSet(l);
    }

    /** Without the gates with this name. */
    public GateSet without(String name) {
        return new GateSet(gates.stream().filter(g -> !g.name().equals(name)).toList());
    }

    /** The gate with {@code replacement}'s name swapped for it (added at the end if there was none). */
    public GateSet replace(Gate replacement) {
        var l = new ArrayList<Gate>();
        boolean found = false;
        for (var g : gates) {
            if (g.name().equals(replacement.name())) {
                l.add(replacement);
                found = true;
            } else {
                l.add(g);
            }
        }
        if (!found) l.add(replacement);
        return new GateSet(l);
    }

    void beginLoop(GateContext ctx) {
        for (var g : gates) g.beginLoop(ctx);
    }

    void accepted(PoseObservation o, GateContext ctx) {
        for (var g : gates) g.accepted(o, ctx);
    }

    /** The verdict on one candidate. */
    public record Verdict(boolean accepted, String gate, String reason) {
        static final Verdict ACCEPTED = new Verdict(true, "", "");
    }

    /** Judges one candidate (in order; the first rejection wins). */
    public Verdict judge(PoseObservation o, GateContext ctx) {
        for (var g : gates) {
            String r = g.reject(o, ctx);
            if (r != null) return new Verdict(false, g.name(), r);
        }
        return Verdict.ACCEPTED;
    }

    /** For {@link org.spectrum3847.vision.VisionSystem}: start a loop, and report acceptances. */
    public static final class Driver {
        private final GateSet set;

        public Driver(GateSet set) {
            this.set = set;
        }

        public void beginLoop(GateContext ctx) {
            set.beginLoop(ctx);
        }

        public Verdict judge(PoseObservation o, GateContext ctx) {
            return set.judge(o, ctx);
        }

        public void accepted(PoseObservation o, GateContext ctx) {
            set.accepted(o, ctx);
        }
    }
}
