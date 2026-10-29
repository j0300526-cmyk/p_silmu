"""One outstanding action request, including late acceptance after cancellation."""
import time


class ActionSlot:
    def __init__(self, client, logger):
        self.client = client
        self.logger = logger
        self.busy = False
        self.handle = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.retry_at = 0.0
        self.done = None

    def send(self, goal, done):
        if self.busy or time.monotonic() < self.retry_at or not self.client.server_is_ready():
            return False
        self.busy = True
        self.handle = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.done = done
        try:
            future = self.client.send_goal_async(goal)
            future.add_done_callback(self._accepted)
        except Exception as exc:
            self._finish(None, str(exc))
        return True

    def _accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                self._finish(None, 'Goal rejected')
                return
            self.handle.get_result_async().add_done_callback(self._result)
            self._cancel_if_needed()
        except Exception as exc:
            # The server may have accepted an unacknowledged goal. Do not send more.
            self.logger.error('Action acceptance failed; restart after checking controller: ' + str(exc))

    def _result(self, future):
        try:
            response = future.result()
        except Exception as exc:
            # An unknown result is not proof that the controller stopped.
            self.logger.error('Action result unavailable; command path locked: ' + str(exc))
            return
        self._finish(response, None)

    def _finish(self, response, error):
        callback = self.done
        was_cancelled = self.cancel_requested
        self.busy = False
        self.handle = None
        self.done = None
        self.retry_at = (time.monotonic() + 0.5 if error or response is None
                         or response.status != 4 else 0.0)
        if callback:
            callback(response, error, was_cancelled)

    def cancel(self):
        if self.busy:
            self.cancel_requested = True
            self._cancel_if_needed()

    def _cancel_if_needed(self):
        if self.cancel_requested and self.handle and not self.cancel_sent:
            self.cancel_sent = True
            try:
                self.handle.cancel_goal_async().add_done_callback(self._cancel_response)
            except Exception as exc:
                self.logger.error('Cancellation failed: ' + str(exc))

    def _cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.logger.warn('Cancellation not accepted; awaiting terminal result')
        except Exception as exc:
            self.logger.error('Cancellation response unavailable: ' + str(exc))
