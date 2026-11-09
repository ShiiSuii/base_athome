"""Control de acciones y escucha de stop de Restaurant, sin dependencias ROS.

Una respuesta a cancel_goal NO confirma que la base se haya detenido. La
operacion conserva su handle (o su aceptacion pendiente) hasta recibir un
resultado terminal. Si el servidor deja de responder, no se envia otro goal
ni se declara terminada la navegacion.
"""

import re
import threading
import time


def is_stop_command(text: str) -> bool:
    """Reconoce la palabra stop tambien con mayusculas y puntuacion de ASR."""
    return "stop" in re.findall(r"\w+", (text or "").casefold())


class NavigationStopMonitor:
    """Escucha no bloqueante; usa y cancela solamente su propio pedido de voz."""

    def __init__(self, voice, stop_fn):
        self.voice = voice
        self.stop_fn = stop_fn
        self.request_id = None
        self.stopped = False

    def poll(self):
        if self.stopped:
            return
        if self.request_id is None:
            self.request_id = self.voice.request(prompt="Stop.", quiet=True, start_timeout=2.0, max_speech=2.0)   # solo se loguea si oye algo
        result = self.voice.poll(self.request_id)
        if result is not None:
            self.request_id = None
            if result.ok and is_stop_command(result.text):
                self.stopped = True
                self.stop_fn()

    def close(self):
        if self.request_id is not None:
            self.voice.cancel(self.request_id)
            self.request_id = None


class _Operation:
    def __init__(self, deadline_s, accept_deadline_s, stop_requested_fn):
        self.deadline_s = deadline_s
        self.accept_deadline_s = accept_deadline_s
        self.stop_requested_fn = stop_requested_fn
        self.terminal = threading.Event()
        self.goal_handle = None
        self.result_future = None
        self.execution_deadline_s = None
        self.status = None
        self.cancel_requested = False
        self.next_cancel_s = 0.0
        self.next_result_s = 0.0


class RestaurantNavigation:
    """Un unico goal, registrado antes del envio y drenado antes de retornar.

    deadline_s es absoluto, con el mismo reloj monotonic que RestaurantSession.
    wait_fn permite probar todos los plazos sin dormir ni crear nodos ROS.
    """

    # Valores terminales definidos por action_msgs/GoalStatus.
    TERMINAL_STATUSES = (4, 5, 6)
    SUCCEEDED = 4

    def __init__(self, client, warn_fn=lambda _: None, clock_fn=time.monotonic, wait_fn=None):
        self.client = client
        self.warn_fn = warn_fn
        self.clock_fn = clock_fn
        self.wait_fn = wait_fn or (lambda event, seconds: event.wait(seconds))
        self._lock = threading.RLock()
        self._operation = None
        self._stopping = False
        self.last_status = None

    @property
    def active_goal_handle(self):
        with self._lock:
            return self._operation.goal_handle if self._operation else None

    @property
    def pending(self):
        with self._lock:
            return self._operation is not None

    def reset(self):
        """Solo un nuevo inicio explicito puede quitar el latch de cancelacion."""
        with self._lock:
            if self._operation is not None:
                return False
            self._stopping = False
            return True

    def cancel(self):
        with self._lock:
            self._stopping = True
            if self._operation:
                self._request_cancel(self._operation)

    def _request_cancel(self, op):
        # Con _lock tomado, incluso cuando stop precede a la aceptacion.
        op.cancel_requested = True
        if (op.goal_handle is None or op.terminal.is_set()
                or self.clock_fn() < op.next_cancel_s):
            return
        op.next_cancel_s = self.clock_fn() + 1.0
        try:
            future = op.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda fut: self._cancel_response(op, fut))
        except Exception as exc:
            self.warn_fn(f"Cancelacion Nav2 sin confirmar; se reintentara: {exc}")

    def _cancel_response(self, op, future):
        with self._lock:
            try:
                future.result()
            except Exception as exc:
                self.warn_fn(f"Cancelacion Nav2 sin confirmar: {exc}")
            # Ni una aceptacion ni un rechazo de cancelacion liberan el handle.

    def _observe_result(self, op):
        if op.result_future is not None or self.clock_fn() < op.next_result_s:
            return
        op.next_result_s = self.clock_fn() + 1.0
        try:
            op.result_future = op.goal_handle.get_result_async()
            op.result_future.add_done_callback(lambda fut: self._result(op, fut))
        except Exception as exc:
            op.result_future = None
            self.warn_fn(f"Resultado Nav2 sin confirmar; se conserva el goal: {exc}")
            self._request_cancel(op)

    def _accepted(self, op, future, timeout_s):
        with self._lock:
            try:
                handle = future.result()
            except Exception as exc:
                # No demuestra rechazo: la solicitud pudo llegar al servidor.
                self.warn_fn(f"Aceptacion Nav2 desconocida; operacion retenida: {exc}")
                op.cancel_requested = True
                return
            if handle is None:
                self.warn_fn("Aceptacion Nav2 sin handle; operacion retenida.")
                op.cancel_requested = True
                return
            if not handle.accepted:
                op.terminal.set()
                return
            op.goal_handle = handle
            now = self.clock_fn()
            op.execution_deadline_s = now + timeout_s
            if (self._stopping or op.stop_requested_fn() or now >= op.deadline_s
                    or now >= op.accept_deadline_s):
                op.cancel_requested = True
            self._observe_result(op)
            if op.cancel_requested:
                self._request_cancel(op)

    def _result(self, op, future):
        with self._lock:
            try:
                status = future.result().status
                if status not in self.TERMINAL_STATUSES:
                    raise ValueError(f"estado no terminal: {status}")
            except Exception as exc:
                op.result_future = None
                self.warn_fn(f"Resultado Nav2 sin confirmar; se conserva el goal: {exc}")
                self._request_cancel(op)
                return
            now = self.clock_fn()
            if (op.stop_requested_fn() or now >= op.deadline_s
                    or now >= op.execution_deadline_s):
                op.cancel_requested = True
            op.status = status
            op.terminal.set()

    def navigate(self, goal, *, deadline_s, timeout_s, stop_requested_fn, monitor_fn=lambda: None):
        with self._lock:
            self.last_status = None
            if self._stopping or self._operation is not None:
                return False
            op = _Operation(deadline_s, self.clock_fn() + 10.0, stop_requested_fn)
            self._operation = op

        def check_interrupt():
            try:
                monitor_fn()
            except Exception as exc:
                self.warn_fn(f"Monitor de stop fallo; cancelando Nav2: {exc}")
                self.cancel()
            with self._lock:
                now = self.clock_fn()
                expired = (now >= op.deadline_s or
                           (op.goal_handle is None and now >= op.accept_deadline_s) or
                           (op.execution_deadline_s is not None and now >= op.execution_deadline_s))
                if self._stopping or stop_requested_fn() or expired:
                    if not op.cancel_requested:
                        self.warn_fn("Stop o plazo Nav2 alcanzado; esperando resultado terminal.")
                    self._request_cancel(op)
                if op.goal_handle is not None and not op.terminal.is_set():
                    self._observe_result(op)
                    if op.cancel_requested:
                        self._request_cancel(op)

        try:
            server_deadline = min(deadline_s, self.clock_fn() + 5.0)
            while True:
                check_interrupt()
                if op.cancel_requested or self.clock_fn() >= server_deadline:
                    op.terminal.set()  # No se envio ninguna solicitud.
                    return False
                try:
                    if self.client.wait_for_server(timeout_sec=max(0.0, min(0.1, server_deadline - self.clock_fn()))):
                        break
                except Exception as exc:
                    self.warn_fn(f"Servidor Nav2 no disponible: {exc}")
                    op.terminal.set()  # Todavia no se envio ninguna solicitud.
                    return False

            with self._lock:
                # Lineariza envio y stop; la operacion ya es visible para cancel().
                if self._stopping or stop_requested_fn() or self.clock_fn() >= deadline_s:
                    op.terminal.set()
                    return False
                op.accept_deadline_s = self.clock_fn() + 10.0
                try:
                    future = self.client.send_goal_async(goal)
                    future.add_done_callback(lambda fut: self._accepted(op, fut, timeout_s))
                except Exception as exc:
                    self.warn_fn(f"Envio Nav2 incierto; operacion retenida: {exc}")
                    op.cancel_requested = True

            # El vencimiento cancela, pero NO se toma como confirmacion de fin.
            while not op.terminal.is_set():
                check_interrupt()
                self.wait_fn(op.terminal, 0.05)
            with self._lock:
                interrupted = op.cancel_requested or self._stopping or stop_requested_fn()
                self.last_status = None if interrupted else op.status
                return not interrupted and op.status == self.SUCCEEDED
        finally:
            with self._lock:
                if op.terminal.is_set() and self._operation is op:
                    self._operation = None
