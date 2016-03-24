# -*- coding: utf-8 -*-
"""

.. moduleauthor:: Eric Cleese <eric@python.invalid>
.. moduleauthor:: John Idle <john@python.invalid>
.. note::

   An example of intersphinx is this: you **cannot** use :mod:`pickle` on this class.


"""


class Delta(object):
    """We use this as a public class example class.

      I don't **know** what's going *on*::

       print(ola)

      hi

    """
    def __init__(self):
        super(Delta, self).__init__()

    def move(self, y, x=1, *args):
        """This function prints hello with a name

        :param str sender: The person sending the message
        :param str recipient: The recipient of the message
        :param str message_body: The body of the message
        :param priority: The priority of the message, can be a number 1-5
        :type priority: integer or None
        :return: the message id
        :rtype: int
        :raises ValueError: if the message_body exceeds 160 characters
        :raises TypeError: if the message_body is not a basestring

        >>> print_hello_with_name('foo')
        Hello, foo

        """
        pass
