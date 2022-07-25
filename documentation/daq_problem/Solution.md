```c++
#include <algorithm>    // std::reverse
void ReverseChannelSamples(const unsigned int num_channels,
const unsigned int samples_per_channel, float64* buffer) {

  for (auto channel : num_channels) {
    std::reverse(buffer + (samples_per_channel * i),
                 buffer + (samples_per_channel*(i+1) - 1));
  }
}
```
