import enum
import time


class ReplyId(enum.IntEnum):
    """Reply codes for messages read from the low-level controller.

    The values and names are from email from Julen 2021-02-19.
    There will presumably be something in a .h file.
    """

    CMD_ACKNOWLEDGED = 1
    CMD_REJECTED = 2
    CMD_SUCCEEDED = 3
    CMD_FAILED = 4
    CMD_SUPERSEDED = 5


foo = frozenset((ReplyId.CMD_ACKNOWLEDGED, ReplyId.CMD_SUPERSEDED))


def time_in(niter=1000000):
    t0 = time.monotonic()
    for i in range(niter):
        5 in foo
    dt = time.monotonic() - t0
    print(
        f"time/iter={dt / niter:0.3g}",
    )


def time_or(niter=1000000):
    t0 = time.monotonic()
    for i in range(niter):
        5 == ReplyId.CMD_ACKNOWLEDGED or 5 == ReplyId.CMD_SUPERSEDED
    dt = time.monotonic() - t0
    print(
        f"time/iter={dt / niter:0.3g}",
    )


time_in()
time_or()
