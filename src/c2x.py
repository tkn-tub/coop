# Copyright (c) 2021 Martin Strunz <strunz@campus.tu-berlin.de>
# Copyright (c) 2021 Julian Heinovski <heinovski@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import numpy as np

import parameter as par
from message import Message


class C2X:
    """
    Class for a (simple) car 2 everything communication system.
    """

    def __init__(self, seed):
        np.random.seed(int(seed))
        print(f'C2X initialized with seed {seed}')
        self.queue = []
        self.delay = []
        self.id = 0

    def send(self, src, dest, msg: Message, data, step):
        """
        Enqueues msg from src to dest to the messaging queue.
        """
        # Do not receive before this step
        rec_step = 0
        if par.MEAN_STEP_DURATION_MSG_DELIVERY > 0:
            rec_step = int(np.random.exponential(par.MEAN_STEP_DURATION_MSG_DELIVERY))
        self.delay.append(rec_step)
        # Define step of message reception:
        self.queue.append({'id': self.id,
                           'src': src,
                           'dest': dest,
                           'msg': msg,
                           'data': data,
                           'receive-min-step': step + rec_step
                           })

    def receive(self, dest, step):
        """
        Returns all messages in the queue that have destination dest.
        """
        result = [msg for msg in self.queue if msg['dest'] == dest and msg['receive-min-step'] <= step]
        self.queue = [
            msg for msg in self.queue
            if msg['dest'] != dest
            or (msg['dest'] == dest and msg['receive-min-step'] > step)  # noqa 503
        ]
        return result

    def delete_all(self):
        """
        Deletes all messages in the queue.
        :return:
        """
        self.queue.clear()
