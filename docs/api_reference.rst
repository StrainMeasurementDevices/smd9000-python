.. module:: smd9000

Python API Reference
==========================================

Package Information
---------------------------------

Logging
++++++++++++++++++++++++
All logging for this module is done with Python's `logging <https://docs.python.org/3/howto/logging.html>`_ module.
There are two named loggers created by this package: one under the name *"SMD9000"*, and the other under the name *"SMD9000_UART"* which logs all received and sent UART commands.

Thread Safety
++++++++++++++++++++++++
The main :class:`SMD9000` class employs Python's `Lock Objects <https://docs.python.org/3/library/threading.html#lock-objects>`_ on any serial communication to ensure thread safety.
If the user application has a user interface element, it is recommended to place any :class:`SMD9000` function calls inside of a thread in order to not block the main user interface.


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
