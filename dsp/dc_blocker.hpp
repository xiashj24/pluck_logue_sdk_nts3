#pragma once

// H(z) = (1 - z^{-1}) / (1 - R*z^{-1}), R = 1 - 20/sr
// Removes DC offset and sub-Hz drift without affecting audio content.
class DcBlocker {
public:
  void reset(float samplerate) noexcept {
    pole = 1.f - 20.f / samplerate;
    x1 = 0.f;
    y1 = 0.f;
  }

  float process_sample(float x) noexcept {
    float y = x - x1 + pole * y1;
    x1 = x;
    y1 = y;
    return y;
  }

private:
  float pole = 0.f;
  float x1 = 0.f;
  float y1 = 0.f;
};
