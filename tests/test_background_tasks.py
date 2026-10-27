"""Slow jobs, cancellation and result delivery without a native window."""

from __future__ import annotations

import threading
import time
import unittest
import weakref

from aurora.gui.background_tasks import BackgroundTasks


class BackgroundTaskTests(unittest.TestCase):
    def test_cancelled_dialog_releases_callback_owner_and_receives_no_result(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)

        class Dialog:
            def ready(self, result, error):
                raise AssertionError("A cancelled dialog received a worker result")

        dialog = Dialog()
        reference = weakref.ref(dialog)

        def work(cancelled):
            entered.set()
            release.wait(2)
            finished.set()
            return "late result"

        tasks.submit("dialog", work, dialog.ready)
        self.assertTrue(entered.wait(1))
        del dialog
        tasks.cancel("dialog")
        self.assertIsNone(reference())
        release.set()
        self.assertTrue(finished.wait(1))
        tasks.drain()

    def test_close_releases_ui_callback_owners_while_a_worker_is_still_busy(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)

        class Receiver:
            def result(self, value, error):
                raise AssertionError("Closed UI received a result")

        owner = Receiver()
        reference = weakref.ref(owner)

        def slow(cancelled):
            entered.set()
            release.wait(3)

        tasks.submit("scan", slow, owner.result)
        self.assertTrue(entered.wait(1))
        del owner
        tasks.close()
        self.assertIsNone(reference())

    def test_repeated_refresh_keeps_only_the_latest_pending_scan(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        release = threading.Event()
        self.addCleanup(release.set)
        entered = [threading.Event(), threading.Event()]
        for index in range(2):

            def wait(cancelled, index=index):
                entered[index].set()
                release.wait(3)

            tasks.submit(f"blocked{index}", wait, lambda value, error: None)
        self.assertTrue(all(event.wait(1) for event in entered))
        results, errors = [], []

        def ready(value, error):
            (errors if error else results).append(error or value)

        for folder in range(100):
            tasks.submit("scan", lambda cancelled, folder=folder: folder, ready)
        release.set()
        deadline = time.monotonic() + 2
        while not results and time.monotonic() < deadline:
            tasks.drain()
            time.sleep(0.001)
        self.assertEqual(errors, [])
        self.assertEqual(results, [99])

    def test_full_queue_does_not_cancel_a_result_whose_replacement_was_rejected(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        release = threading.Event()
        self.addCleanup(release.set)
        started = [threading.Event(), threading.Event()]
        results = []

        def work(index):
            def blocked(cancelled):
                started[index].set()
                release.wait(3)
                return index

            return blocked

        for index in range(2):
            tasks.submit(str(index), work(index), lambda value, error: results.append(value))
        self.assertTrue(all(event.wait(1) for event in started))
        for index in range(8):
            tasks.submit(f"queued{index}", lambda cancelled: None, lambda value, error: None)
        errors = []
        tasks.submit(
            "0", lambda cancelled: "replacement", lambda value, error: errors.append(error)
        )
        self.assertRegex(str(errors[0]), "busy")
        release.set()
        deadline = time.monotonic() + 2
        while len(results) < 2 and time.monotonic() < deadline:
            tasks.drain()
            time.sleep(0.001)
        self.assertCountEqual(results, [0, 1])

    def test_slow_job_does_not_block_caller_and_callbacks_run_on_the_caller(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        results = []

        def work(cancelled):
            entered.set()
            release.wait(2)
            return "scanned"

        started = time.monotonic()
        tasks.submit(
            "scan",
            work,
            lambda result, error: results.append((result, error, threading.get_ident())),
        )
        self.assertLess(time.monotonic() - started, 0.1)
        self.assertTrue(entered.wait(1))
        tasks.drain()
        self.assertEqual(results, [])
        release.set()
        deadline = time.monotonic() + 1
        while not results and time.monotonic() < deadline:
            tasks.drain()
            time.sleep(0.001)
        self.assertEqual(results, [("scanned", None, threading.get_ident())])

    def test_replaced_scan_cannot_overwrite_the_new_folder_result(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        results = []

        def old_scan(cancelled):
            entered.set()
            release.wait(2)
            return "old folder"

        tasks.submit("scan", old_scan, lambda result, error: results.append(result))
        self.assertTrue(entered.wait(1))
        tasks.submit(
            "scan", lambda cancelled: "new folder", lambda result, error: results.append(result)
        )
        release.set()
        deadline = time.monotonic() + 1
        while not results and time.monotonic() < deadline:
            tasks.drain()
            time.sleep(0.001)
        tasks.drain()
        self.assertEqual(results, ["new folder"])

    def test_worker_failure_reaches_result_callback(self):
        tasks = BackgroundTasks()
        self.addCleanup(tasks.close)
        results = []

        def failed(cancelled):
            raise OSError("folder unavailable")

        tasks.submit("scan", failed, lambda result, error: results.append(error))
        deadline = time.monotonic() + 1
        while not results and time.monotonic() < deadline:
            tasks.drain()
            time.sleep(0.001)
        self.assertIsInstance(results[0], OSError)
        self.assertEqual(str(results[0]), "folder unavailable")


if __name__ == "__main__":
    unittest.main()
