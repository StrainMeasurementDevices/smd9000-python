Examples
========================================

Basic Reading
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python

    import smd9000

    sensor = smd9000.SMD9000()
    if sensor.connect('COM39'):
        print(sensor.read())
    sensor.disconnect()

Getting Sensor Information
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python

    import smd9000

    with smd9000.SMD9000('COM39') as sensor:
        print(sensor.read_info())

Reading a datastream with a callback function
++++++++++++++++++++++++++++++++++++++++++++++++
.. code-block:: python

    import smd9000
    import time

    def read_data(in_data):
        for d in in_data:
            print(f"Flow: {d.flow}")

    with smd9000.SMD9000('COM39') as sensor:
        sensor.set_stream_rate(250)
        sensor.start_data_stream(read_data)
        time.sleep(2)
        sensor.stop_data_stream()
