package org.hivevisionftc.vision;

/**
 * A class that holds all the different classifiers for different detections
 */
public class Classifier {
    // You can repeat this structure for different types of detections, if you want (eg. next season? Just copy everything in the enum and change names)
    public enum BiobuzzDetections { // https://www.baeldung.com/java-enum-values
        YELLOW_POLLEN(0),
        RED_NECTAR(1),
        BLUE_NECTAR(2),
        UNKNOWN(-1);

        private final int id;
        BiobuzzDetections(int id) {
            this.id = id;
        }

        public static BiobuzzDetections classifyFromId(int id) {
            for(BiobuzzDetections detectionType : BiobuzzDetections.values()) { // for each enum in the enum BiobuzzDetections
                if(id == detectionType.id) return detectionType; // If the selected enum's id is equal to peram id, return the selected enum
            }

            return UNKNOWN; // If none match, return unknown
        }
    }
}
