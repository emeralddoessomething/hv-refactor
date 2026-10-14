package org.hivevisionftc.vision;

import java.util.List;

public interface DetectionSource {
    List<Detection> latest();

    long stalenessMs();
}
