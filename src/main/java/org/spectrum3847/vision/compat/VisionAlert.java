package org.spectrum3847.vision.compat;

import edu.wpi.first.wpilibj.Alert;

/** An alert on the dashboard (WPILib 2026: Alert with AlertType). */
public final class VisionAlert {
    public enum Level {
        ERROR,
        WARNING,
        INFO
    }

    private final Alert alert;

    public VisionAlert(String group, String text, Level level) {
        alert =
                new Alert(
                        group,
                        text,
                        switch (level) {
                            case ERROR -> Alert.AlertType.kError;
                            case WARNING -> Alert.AlertType.kWarning;
                            case INFO -> Alert.AlertType.kInfo;
                        });
    }

    public void set(boolean active) {
        alert.set(active);
    }

    public void setText(String text) {
        alert.setText(text);
    }
}
