.. module:: smd9000

Python API Reference
==========================================

Package Information
---------------------------------

Logging
++++++++++++++++++++++++
All logging for this module is done with Python's `logging <https://docs.python.org/3/howto/logging.html>`_ module under the name *"SMD9000"*.

Thread Safety
++++++++++++++++++++++++
The main :class:`SMD9000` class employs Python's `Lock Objects <https://docs.python.org/3/library/threading.html#lock-objects>`_ on any serial communication to ensure thread safety.
If the user application has a user interface element, it is recommended to place any :class:`SMD9000` function call inside of a thread in order to not block the main user interface.


Package Reference
---------------------------------

CQV Main Class
++++++++++++++++++++++++

.. autoclass:: SMD9000
  :members:
  :undoc-members:

CQV Data Class
++++++++++++++++++++++++
.. autoclass:: SMD9000Revisions
  :members:
  :undoc-members:

.. autoclass:: SMD9000Data
  :members:
  :undoc-members:

CQV Exceptions
++++++++++++++++++++++++
.. autoexception:: SMD9000ReadException
  :members:
