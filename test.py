def read(self, sensors):
      sensorValues = []

      for i, sensor in enumerate(sensors):
        sensorValues[i] = 4095
        # make sensor line an output (drives low briefly, but doesn't matter)
        sensor.init(sensor.OUT)
        # drive sensor line high
        sensor.value(1)
      

      time.sleep(.00001) # charge lines for 10 us

      # record start time before the first sensor is switched to input
      # (similarly, time is checked before the first sensor is read in the
      # loop below)
      startTime = time.tics_us()
      time = 0;

      for i in range(0, len(sensors)):
        # make sensor line an input (should also ensure pull-up is disabled)
        sensor.init(sensor.IN)


      while (time < maxValue):
        time = ticks_diff(ticks_us() - startTime)
        for i in range(0, len(sensors)):
          if (sensors[i].value() == 0) and (time < sensorValues[i]):
            # record the first time the line reads low
            sensorValues[i] = time
        



