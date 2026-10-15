package org.hivevisionftc.vision;

public enum BiobuzzClasses {
    POLLEN(0),
    NECTAR_RED(1),
    NECTAR_BLUE(2),
    UNKNOWN(Integer.MAX_VALUE); // For safety
    private final int id;

    BiobuzzClasses(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }

    public static BiobuzzClasses fromId(int id) {
        for (BiobuzzClasses clazz : BiobuzzClasses.values()) {
            if (clazz.getId() == id) {
                return clazz;
            }
        }
        return UNKNOWN;
    }
}
