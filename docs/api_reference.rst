.. module:: smd9000

Python API Reference
==========================================

Package Information
---------------------------------

Logging
++++++++++++++++++++++++
All logging for this module is done with Python's `logging <https://docs.python.org/3/howto/logging.html>`_ module.
There is parent logger created by this package under the name *"smd9000"*.

A child logger under the name *"smd9000.uart"* logs all incoming and outgoing serial communication commands. This can be
latched on in order to log or showcase all serial communication.

Thread Safety
++++++++++++++++++++++++
The main :class:`SMD9000` class employs Python's `Lock Objects <https://docs.python.org/3/library/threading.html#lock-objects>`_ on any serial communication to ensure thread safety.
If the user application has a user interface element, it is recommended to place any :class:`SMD9000` function calls inside of a thread in order to not block the main user interface.

Context Manager
++++++++++++++++++++++++
The class :class:`SMD9000` can be called from a context manager, which can be done by using the `with` statement.
Below is an example

.. code-block:: python

    from smd9000 import SMD9000

    with SMD9000('COMxx') as sensor:
        sensor.get_info()

Package Reference
---------------------------------

SMD9000 Main Class
++++++++++++++++++++++++

.. autoclass:: SMD9000

SMD9000 Data Classes
++++++++++++++++++++++++
.. autoclass:: SensorInfo

.. autoclass:: DataSet

.. autoclass:: StatusCode

.. autoclass:: StatusWord

.. autoclass:: StreamFormat

.. autoclass:: available_filter_time

SMD9000 Exceptions
++++++++++++++++++++++++
.. autoexception:: ReadException
.. autoclass:: InvalidProfile
.. autoclass:: NotInCalibrationMode
