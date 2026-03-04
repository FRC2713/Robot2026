package frc2713.lib.drivers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class CANDeviceIdTest {

  @Nested
  class Constructor {

    @Test
    void deviceNumberOnly_usesDefaultBus() {
      CANDeviceId id = new CANDeviceId(5);
      assertEquals(5, id.getDeviceNumber());
      assertEquals("", id.getBus());
    }

    @Test
    void deviceNumberAndBus_storesBoth() {
      CANDeviceId id = new CANDeviceId(3, "canivore");
      assertEquals(3, id.getDeviceNumber());
      assertEquals("canivore", id.getBus());
    }
  }

  @Nested
  class Getters {

    @Test
    void getDeviceNumber_returnsStoredValue() {
      assertEquals(1, new CANDeviceId(1).getDeviceNumber());
      assertEquals(42, new CANDeviceId(42, "bus").getDeviceNumber());
    }

    @Test
    void getBus_returnsStoredValue() {
      assertEquals("", new CANDeviceId(1).getBus());
      assertEquals("canivore", new CANDeviceId(1, "canivore").getBus());
    }
  }

  @Nested
  class Equals {

    @Test
    void sameDeviceAndBus_returnsTrue() {
      CANDeviceId a = new CANDeviceId(2, "canivore");
      CANDeviceId b = new CANDeviceId(2, "canivore");
      assertTrue(a.equals(b));
      assertTrue(b.equals(a));
    }

    @Test
    void sameDeviceDefaultBus_returnsTrue() {
      CANDeviceId a = new CANDeviceId(2);
      CANDeviceId b = new CANDeviceId(2, "");
      assertTrue(a.equals(b));
    }

    @Test
    void differentDeviceNumber_returnsFalse() {
      CANDeviceId a = new CANDeviceId(2, "canivore");
      CANDeviceId b = new CANDeviceId(3, "canivore");
      assertFalse(a.equals(b));
    }

    @Test
    void differentBus_returnsFalse() {
      CANDeviceId a = new CANDeviceId(2, "canivore");
      CANDeviceId b = new CANDeviceId(2, "rio");
      assertFalse(a.equals(b));
    }

    @Test
    void bothDifferent_returnsFalse() {
      CANDeviceId a = new CANDeviceId(1, "a");
      CANDeviceId b = new CANDeviceId(2, "b");
      assertFalse(a.equals(b));
    }
  }
}
