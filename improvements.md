## Improvements: 

The following table outlines the improvements that could be made to our Romi assembly. 

 

| Improvement                          | Implementation Location    | Description |
|--------------------------------------|----------------------------|-------------|
| Integral and Derivative Control      | Motor Feedback Loop        | Implementing integral and derivative control to our feedback loop would eliminate steady state error and increase our response time to zero error. |
| Decrease Bump Sensor Sequence Steps  | Bump Sensor Sequence       | Our bump sensor sequence is currently seven steps long and can be easily decreased. Decreasing these steps would make our code more efficient and potentially allow the Romi to run that section of the course faster. |
| Improved Logic for Finish Sequence   | Finish Sequence, Throughout| Currently, our finish sequence consists of line sensing and a “hard” coded segment. This is possible because we know the configuration of the course beforehand. Moving forward, we could collect timestamped data from encoders and the IMU in order to obtain a global position relative to our start box. Using this data we would find our current coordinate and run a calculation using path prediction in order to find the quickest way to return to the start. |
| Improve Wiring Flow                  | Hardware                   | The wires on our Romi are currently messy and could be cleaned up. This would allow us to more easily see how each sensor is connected to the microcontroller, as well as making it easier to modify the pin-out definitions and sensors. |
