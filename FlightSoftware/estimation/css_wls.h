#pragma once

/** Body sun heading from 6 orthogonal CSS (winner per axis, cosine invert). */
class CssWls {
public:
    void reset();

    /** css[6] in ±X, ±Y, ±Z order → unit sun_B. Returns false if no sun. */
    bool update(const float css[6], float sun_B[3]);

    static constexpr float kScale = 2.0f;
    static constexpr float kMinIlluminated = 0.05f;
    static constexpr int kNumCss = 6;
};
