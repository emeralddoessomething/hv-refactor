package org.hivevisionftc;

import java.util.List;

public interface DetectionSource {
    List<Detection> latest();

    long stalenessMs();
}
